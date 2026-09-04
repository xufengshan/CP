import numpy as np
import time
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_driver_steer_torque_limits, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.byd.tuning import Tuning

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.frame = 0
    self.last_steer_frame = 0
    self.last_acc_frame = 0

    self.apply_torque_last = 0

    self.mpc_lkas_counter = 0
    self.mpc_acc_counter = 0
    self.eps_fake318_counter = 0
    self.pcm_button_counter = 0

    self.lkas_req_prepare = 0
    self.lkas_active = 0
    self.lat_safeoff = 0
    # HANDSOFF (hands-off EPS protection): track sustained hands-off + wheel motion
    self.handsoff_angle_cnt = 0
    self.handsoff_last_exit = 0.0

    self.steer_softstart_limit = 0
    self.steerRateLimActive = False
    self.steerRateLim = 1.0

    # V9横向100%学习 (2026-09-02): 启停/等红灯保护
    self.lat_inactive_frames = 0   # 启停/等红灯计时 (等红灯LKAS故障: 超15000帧清零重启握手)
    self.soft_start_torque_limit = 0

    self.first_start = True
    self.rfss = 0 # resume from stand still
    self.sss = 0 #stand still state

    self.apply_accel_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # 濡亜鎮滈幒褍鍩楅柈銊ュ瀻 - 娣囨繃瀵旈崢鐔告箒闁槒绶?
    if (self.frame - self.last_steer_frame) >= CarControllerParams.STEER_STEP:
      if self.first_start:
        self.mpc_lkas_counter = int(CS.acc_mpc_state_counter + 1) & 0xF
        self.mpc_acc_counter = int(CS.acc_cmd_counter + 1) & 0xF
        self.eps_fake318_counter = int(CS.eps_state_counter + 1) & 0xF
        self.first_start = False

      apply_torque = 0

      if CC.latActive:
        # 起步恢复：重置启停计时器
        self.lat_inactive_frames = 0
        # HANDSOFF 拟人闪断 (2026-09-04 08:48 用户定: 改回 REF need_steer 判据, 撤销 CP11 偏摆判据回归):
        #   - 弯道/有真实转向需求(OP 期望扭矩大): 方向机在正常工作有持续力矩, 不会因离手报错 -> 不闪断
        #   - 直线/无转向需求(OP 期望扭矩≈0, 仅直行稳住): 离手久了车机15秒安全会退出 -> 按时间周期闪断重置
        # 判据 = CC.actuators.torque (OP 是否有真实转向需求), 不用方向盘偏摆角 (>4° 偏摆闪断是错的/不安全,
        #         直线<4°不闪断→车机15秒退出ACC/弯道修正频繁闪断→扰动ACC状态, 影响纵向/灯闪, 已确认改回)。
        # 触发前提: ACC有效(lkas_active)+行驶有速度(vEgo>0.5m/s)+离手+直线无转向需求。
        now_s = time.time()
        is_handsoff = abs(CS.out.steeringTorque) < 10.0        # 驾驶员离手(没施加力矩)
        need_steer  = abs(CC.actuators.torque) > 0.06          # OP 有真实转向需求(弯道/纠偏)->不闪断
        if self.lkas_active and is_handsoff and not need_steer and CS.out.vEgo > 0.5:
            self.handsoff_angle_cnt += 1
            # 持续(行驶+离手+直线无转向)够 HANDSOFF_PERIOD[0] -> 周期性闪断(lat_safeoff 归零重启ACC), 规避车机15秒离手退出ACC
            if self.handsoff_angle_cnt >= Tuning.HANDSOFF_PERIOD[0] * CarControllerParams.STEER_STEP * 10:
                if now_s - self.handsoff_last_exit > Tuning.HANDSOFF_PERIOD[0]:
                    self.lat_safeoff = 1
                    self.handsoff_last_exit = now_s
                    self.handsoff_angle_cnt = 0
        else:
            self.handsoff_angle_cnt = 0
        # 唐DM: 输出扭矩前额外检查 EPS 实时 LKAS_State prepared(1/2)，避免持续驱动未就绪的 EPS 触发 TorqueFailed 保护。
        # 汉DM: lkas_prepared 来自 LKAS_Prepared 1bit 字段，与旧逻辑等价，保持行为不变。
        # (融合版 2026-09-02: 横向回归 V9 纯净握手, 横向激活/退避由 ESC LKAS_State 握手驱动 + HANDSOFF拟人闪断)
        if self.lkas_active and (not getattr(CS, 'is_tang_dm', False) or CS.lkas_prepared):
          steer_desire = CC.actuators.torque

          if CarControllerParams.USE_STEERING_SPEED_LIMITER:
            rate_limit = np.interp(CS.out.aEgo, [8.3, 27.8], [132, 64])
            delta_rate = CS.steeringRateDegAbs - rate_limit

            if delta_rate < 0:
              self.steerRateLim -= 0.005 * delta_rate
              if delta_rate < -0.05:
                self.steerRateLimActive = False
              if self.steerRateLim > 1.0:
                self.steerRateLim = 1.0
                self.steerRateLimActive = False
            else:
              if self.steerRateLimActive:
                self.steerRateLim -= 0.005 * delta_rate
              else:
                self.steerRateLim = steer_desire
                self.steerRateLimActive = True
              if self.steerRateLim < 0:
                self.steerRateLim = 0

            new_steer_pu = np.clip(steer_desire, -self.steerRateLim, self.steerRateLim)
          else:
            new_steer_pu = steer_desire

          new_steer = int(round(new_steer_pu * CarControllerParams.STEER_MAX))

          if self.steer_softstart_limit < CarControllerParams.STEER_MAX:
            self.steer_softstart_limit = self.steer_softstart_limit + CarControllerParams.STEER_SOFTSTART_STEP
            new_steer = np.clip(new_steer, -self.steer_softstart_limit, self.steer_softstart_limit)

          # V9横向100%学习: 防止OP/LKAS与驾驶员反向抢盘触发EPS TorqueFailed(报 lkas / steerUnavailable)。
          # 现象:过匝道时OP仍在输出横向扭矩,驾驶员强行打方向,两股相反扭矩持续对拉,
          #       EPS被顶到TorqueFailed永久故障(需重启车辆才能恢复)。
          #       OP与驾驶员对抗时, 驾驶员扭矩与命令扭矩方向相反且|驾驶员扭矩|超过允许量时,
          #       视为驾驶员强制接管, 立即将命令归零, 交由apply_driver_steer_torque_limits按速率
          #       限制平滑回零(STEER_DELTA_DOWN), EPS不再被反向顶。同向(驾驶员帮着打)时不干预。
          driver_torque = CS.out.steeringTorque
          if (new_steer != 0 and driver_torque != 0 and
              (new_steer > 0) != (driver_torque > 0) and
              abs(driver_torque) > CarControllerParams.STEER_DRIVER_ALLOWANCE):
            new_steer = 0

          # (2026-09-02 16:4x 用户定: 去除 V9 LOW_SPEED 停车扭矩限幅 -
          #  与 HANDSOFF 闪断冲突: 低速挪车方向盘>4° 且离手时会触发HANDSOFF频繁闪断, 该限幅多余+干扰)

          apply_torque = apply_driver_steer_torque_limits(new_steer, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams)
        else:
          # 官方握手 (BYD0831, 用户 09-01 18:23 定): 等原车ESC回 LKAS_Prepared=1 才激活, 否则先发预备请求
          # (黄金版路试铁证 00000002 seg3-5: 原车ESC 0x318 LKAS_Prepared 全程=1, 握手成立;
          #  OP 0x316 ReqPrepare=1 占8518帧主导 -> 必须先发预备请求等回执, 不能直接激活)
          # (rtA 18:10 seg4/seg5: 真机 ESC 回 LKAS_Prepared=181/230帧 -> 握手可行)
          # 唐DM退避: 已激活但 EPS 撤 prepared(未就绪/已TemporaryFail) → 立即清零,
          # 避免持续驱动未就绪的 EPS 触发 TorqueFailed 保护 (V9 横向正确逻辑)
          if self.lkas_active and not CS.lkas_prepared:
            self.lkas_active = 0.0
          if CS.lkas_prepared:
            self.lkas_active = 1.0
            self.steerRateLimActive = False
            self.steerRateLim = 1.0
            self.lkas_req_prepare = 0
            self.steer_softstart_limit = 0
            self.lat_safeoff = 1
          else:
            self.lkas_req_prepare = 1

      elif self.lat_safeoff:
        if self.apply_torque_last == 0:
          self.lat_safeoff = 0
        apply_torque = apply_driver_steer_torque_limits(0, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams)
      else:
        self.lkas_req_prepare = 0
        self.steerRateLimActive = False
        self.steerRateLim = 1.0
        # V9横向100%学习: 启停场景(CC.latActive因standstill短暂为False)保留lkas_active和
        # steer_softstart_limit, 起步时直接输出扭矩, 避免softstart从0爬升导致前几百毫秒方向盘没力、横向丢失。
        # 时间窗保护: 停车超过 15000 帧(约5分钟@50Hz)后强制清零, 覆盖几乎所有红绿灯场景。
        #
        # EPS TorqueFailed 永久故障保护(v9-BYD seg52复现修复):
        # 根因: BYD EPS 在 CruiseActivated=1 + vEgo<0.3 + OP发lkas_active=1+LKAS_Output=0
        #       的矛盾状态下, 给40ms宽限期后置位TorqueFailed永久故障(需重启车)。
        # 修复: 监测EPS反馈的激活状态, 仅在 EPS仍激活+CruiseAct=1+vEgo<0.3 时立即清零lkas_active释放EPS。
        #       正常stop-and-go中EPS已撤激活, 不触发清零, lkas_active保留, 起步瞬时激活。
        # 唐DM: 无CruiseActivated字段(新DBC用LKAS_State枚举), 用 lkas_state==2(Active) 判断EPS激活。
        self.lat_inactive_frames += 1
        if getattr(CS, 'is_tang_dm', False):
          cruise_activated = CS.lkas_state == 2  # LKAS_State Active
        else:
          cruise_activated = bool(CS.esc_eps.get('CruiseActivated', 0)) if CS.esc_eps else False
        if (CS.out.vEgo <= 0.3 and cruise_activated) or CS.out.vEgo > 0.3 or self.lat_inactive_frames > 15000:
          self.lkas_active = 0
          self.steer_softstart_limit = 0
          self.lat_inactive_frames = 0
        # 注意: 此处不强制同步mpc_laks_active。
        # 原因: else分支覆盖"latActive=False且非safeoff"场景(行驶中横向未激活/低速过渡/握手失败)。
        # 若强制lkas_active=1, OP未控车却发CruiseActivated=1, EPS检测双源冲突→高频steerFaultTemporary
        # →controlsd高频处理+UI高频刷新告警→CPU占满→画面卡死(别人车行驶中偶发)。
        # 等红灯LKAS故障问题(standstill场景)由lat_inactive_frames>15000超时清零+重启握手解决。
        self.soft_start_torque_limit = 0

      self.apply_torque_last = apply_torque

      self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
      self.eps_fake318_counter = int(self.eps_fake318_counter + 1) & 0xF
      self.last_steer_frame = self.frame

      can_sends.append(bydcan.create_steering_control(self.packer, self.CP, CS.cam_lkas,
          self.apply_torque_last, self.lkas_req_prepare, self.lkas_active, CC.hudControl, self.mpc_lkas_counter))

      can_sends.append(bydcan.create_fake_318(self.packer, self.CP, CS.esc_eps,
                                              CS.mpc_laks_output, CS.mpc_laks_reqprepare, CS.mpc_laks_active,
                                              True, self.eps_fake318_counter))

      # ⭐ 纵向我们自己的: ACC_HUD_ADAS (0x32D) 主动广播 (黄金对齐 8/14: 与 0x316/0x318 同频 50Hz)
      #   (用户 09-02: 纵向我们自己的, 保留 create_hud_adas; V9 不发0x32D靠转发, 但我们纵向依赖主动发)
      can_sends.append(bydcan.create_hud_adas(self.packer, self.CP, CS.cam_hud, CS, CC, CC.longActive, self.eps_fake318_counter))


    # ⭐ 黄金对齐 (8/14): 黄金 OP 不主动发 ACC_CMD (0x377) — 黄金任何 src 无此帧
    #   (黄金不做 OP 纵向主动控制, 依赖原车 ACC) → 移除 acc_cmd 主动发送
    #   注: 保留自增计数器避免破坏其他逻辑引用
    if (self.frame + 1 - self.last_acc_frame) >= CarControllerParams.ACC_STEP:
      accel = np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      if CC.longActive:
        stopping = CC.actuators.longControlState == LongCtrlState.stopping
        starting = CC.actuators.longControlState == LongCtrlState.starting
        running = CC.actuators.longControlState == LongCtrlState.pid

        #stopping and stopped
        if stopping and accel < -0.1:
          self.rfss = 0
          self.sss = CS.out.standstill

        #re-starting
        elif starting and accel > 0.1 and CS.mrr_leading_dist > 3:
          self.rfss = CS.out.standstill
          self.sss = 0

        #started
        elif running:
          self.rfss = 0
          self.sss = 0

        # ⭐ L1 末段刹停收敛 (汇报审核 2026-09-02, 黄金 00000002 seg2 t197-203.5 刹停档案实证):
        #   现象 = "停稳点头": 命令已收到 -0.9(t202.6)/≈0(t203.4), 但车 aEgo 仍带 -2.5/-1.8 过冲,
        #   源自车惯量/ESC 执行在末段把减速度冲过头 → 行程最后 ~0.5s 明显"一顿/点头"。
        #   处理 = 仅当 stopping 且已极低速(vEgo<1.0m/s, 距完全停约~0.5m内)时, 把请求向温柔末段值平滑收敛
        #         (不得比 -1.2 更深), 让减速度"缓到底"而非对准停车继续加深。仅在极低速触发, 停车距离影响≈0.
        #   ⚠️ 保守实现: 只在最后 ~0.5m 生效, 不碰中段跟车制动; 若真机回归觉停车偏软/距离拉长可回调 -1.5~-1.8.
        if stopping and CS.out.vEgo < 1.0:
          accel = max(accel, -1.2)   # 末段请求下限 -1.2(缓到底), 抑制过冲

      else:
        accel = 0
        self.sss = 0
        self.rfss = 0

      self.mpc_acc_counter = int(self.mpc_acc_counter + 1) & 0xF

      # ⭐ 黄金对齐 (8/14): ACC_CMD = 0x32E (814), 黄金 OP 发此帧做纵向 (src128 3000帧)
      #   保留 acc_cmd 主动发送 — 黄金 OP 也发 0x32E 纵向指令
      can_sends.append(bydcan.acc_cmd(self.packer, self.CP, CS.cam_acc,
                                     CS.mrr_leading_dist,
                                     accel, self.rfss, self.sss, CC.longActive))

      self.apply_accel_last = accel
      self.last_acc_frame = self.frame + 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = float(self.apply_accel_last)
    new_actuators.steeringAngleDeg = float(CS.out.steeringAngleDeg)

    # 🔧 2026-09-01 18:58 移除 0x3B0 恒发假主开关 (组合方案 a — 用户定)
    #   根因: 真机 fwd_hook 已把原车方向盘按钮 (bus0 0x3B0) 转发到 bus2 (原车ACC控制器),
    #         再恒发 create_pcm_buttons 假主开关 (BTN_TOGGLE_ACC_OnOff=1, 20Hz) → 同一 bus2 两路 0x3B0 冲突
    #         → 原车ACC收到混乱按钮状态 → 上下键(启动)不稳定/被截断 + 限速识别地址被打开 → 限速标记常亮
    #   修复: 停止恒发假主开关, 靠 fwd 转发原车按钮 (原车自主控制按键, 纵向零改动)
    #   (参考 V9: fwd block 0x3B0 + create_button_fwd 转发; 但那是 V9 纵向按钮注入, 会污染我们好的纵向, 不学)
    self.frame += 1
    return new_actuators, can_sends
