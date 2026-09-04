import copy
import numpy as np

#-----disabled for pytests-----
#from datetime import datetime, timedelta
#import subprocess
#from openpilot.common.time_helpers import system_time_valid
#from openpilot.common.swaglog import cloudlog

#from opendbc.can.can_define import CANDefine
#from opendbc.can.parser import CANParser
from opendbc.can import CANDefine, CANParser

from opendbc.car.common.conversions import Conversions as CV
#from opendbc.car.common.numpy_fast import mean
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CanBus, LKASConfig, CarControllerParams, CAR
from opendbc.car.byd.tuning import Tuning

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
    def __init__(self, CP):
        super().__init__(CP)

        can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

        self.shifter_values = can_define.dv["DRIVE_STATE"]["Gear"]

        self.speed_kph = 0

        self.mpc_lkas_config = 0

        self.acc_hud_adas_counter = 0
        self.acc_mpc_state_counter = 0
        self.acc_cmd_counter = 0

        self.eps_warning = False
        # EPS angle/rate abnormal counters (consumed by Tuning.EPS_ANGLE_*_WARNING_CNT)
        self.eps_angle_exceed_cnt = 0
        self.eps_angle_speed_cnt = 0

        self.acc_active_last = False
        self.low_speed_alert = False
        self.lkas_allowed_speed = False

        self.lkas_prepared = False  #318, EPS to OP
        self.lkas_state = 0  #318 LKAS_State 2bit: 0 NotReady/1 Ready/2 Active/3 TempFail
        self.is_tang_dm = CP.carFingerprint == CAR.BYD_TANG_DM  # V9横向100%学习: 区分唐DM(LKAS_State枚举)分支
        self.acc_state = 0
        self.adas_set_dist = 0

        self.mpc_laks_output = 0
        self.mpc_laks_active = False
        self.mpc_laks_reqprepare = False

        #self.prev_angle = 0

        self.cam_lkas = 0
        self.cam_acc = 0
        self.cam_hud = 0
        self.esc_eps = 0

        self.setTimeDelay = 100

        self.mrr_leading_dist = 199  # 雷达不可用兜底(避免 bydcan.acc_cmd 的 mrr_leaddist>3 条件失效导致纵向不发送)

        self.btn_acc_cancel = 0
        self.btn_acc_set_reset = 0
        self.btn_acc_dist_inc = 0
        self.btn_acc_dist_dec = 0

        self.prev_steeringAngleDeg = 0
        #self.steeringRate = 0.0
        self.steeringRateDegAbs = 0
        self.esp_lkas_CruiseActivated = False



    def update(self, can_parsers) -> structs.CarState: # type: ignore
        cp = can_parsers[Bus.pt]
        cp_cam = can_parsers[Bus.cam]

        ret = structs.CarState()

        # 纵向优化 (汇报审核 2026-09-04, 用户批 A 修正): mrr_leading_dist 从恒199 死值 → 真实前车距离。
        #   DBC byd_han_dmev_2020 的 RADAR_MAIN_TARGET(0x109) MainDist 信号 = (0.5,-4) 已由 CANParser
        #   按 factor/offset 解码为真实米数(0x109 主目标 100% 验证)。故此处 _d_lead 直接用 MainDist 解码值,
        #   **不得再套 0.5*b7-4**(那是 radar_interface 自算 raw bit 的公式; 若对解码值二次缩放会把车距
        #   算成一半再减4 → mrr 值错乱 → acc_cmd 门控 mrr_leaddist>3 反复开合 → ACC 一启动/跟车就闪断,
        #   07:33 用户实测 ACC 灯亮闪回归即此因, 已修)。有效距离取真实值(1~120m), 无/越界回 199 兜底。
        cp_radar = can_parsers.get(Bus.radar)
        if cp_radar is not None and 'RADAR_MAIN_TARGET' in cp_radar.vl:
            _md = float(cp_radar.vl['RADAR_MAIN_TARGET'].get('MainDist', 0.0) or 0.0)
            _d_lead = float(_md)   # MainDist 已由 DBC factor(0.5)/offset(-4) 解码 = 真实米, 直接用(勿二次缩放)
            self.mrr_leading_dist = int(round(_d_lead)) if (1.0 <= _d_lead <= 120.0) else 199
        # 若 can_parsers 无 Bus.radar 键 / 无雷达帧, 保持原 199 兜底不变

        # 唐DM: 0x318 bit0-1 = LKAS_State 2bit枚举 (0 NotReady/1 Ready/2 Active/3 TempFail)
        self.lkas_state = int(cp.vl["ACC_EPS_STATE"]["LKAS_State"])
        self.lkas_prepared = self.lkas_state in (1, 2)  # Ready/Active 算 prepared
        self.esp_lkas_CruiseActivated = (self.lkas_state == 2)  # CruiseActivated = LKAS_State Active(2)

        self.mpc_lkas_config = int(cp_cam.vl["ACC_MPC_STATE"]["LKAS_Config"])
        lkas_config_isAccOn = (self.mpc_lkas_config != LKASConfig.DISABLE)
        lkas_isMainSwOn = bool(cp.vl["PCM_BUTTONS"]["BTN_TOGGLE_ACC_OnOff"])
        self.lkas_isMainSwOn = bool(cp.vl["PCM_BUTTONS"]["BTN_TOGGLE_ACC_OnOff"])
        # 原车摄像头被 CP(C3X) 替代后, 其 ACC_HUD_ADAS 持续广播 ERROR(AccState=7, AccOn1=0),
        # 不可作为 ACC 可用性/激活判断依据 (否则 available=False → wrongCarMode → 无法 engage)。
        # 参考现代 (Hyundai) 方法论: available 看主开关(MainMode_ACC) + 配置, 不读被替代的摄像头 ERROR。
        # AccOn1 跟随主开关 (用户按 ACC 主开关即可 engage)
        lkas_hud_AccOn1 = lkas_isMainSwOn
        self.acc_state = cp_cam.vl["ACC_HUD_ADAS"]["AccState"]
        # 原车 ERROR(7) 且主开关已开 → 说明原车视觉被替代, 其 ERROR 不可信, 视为未激活(0), 不误报转向故障
        if self.acc_state == 7 and lkas_isMainSwOn:
          self.acc_state = 0
        self.adas_set_dist = cp_cam.vl["ACC_HUD_ADAS"]["SetDistance"]

        prev_btn_acc_cancel = self.btn_acc_cancel
        prev_btn_acc_set_reset = self.btn_acc_set_reset
        prev_btn_acc_dist_inc = self.btn_acc_dist_inc
        prev_btn_acc_dist_dec = self.btn_acc_dist_dec

        self.btn_acc_cancel = cp.vl["PCM_BUTTONS"]["BTN_AccCancel"]
        self.btn_acc_set_reset = cp.vl["PCM_BUTTONS"]["BTN_AccUpDown_Cmd"]
        self.btn_acc_dist_inc = cp.vl["PCM_BUTTONS"]["BTN_AccDistanceIncrease"]
        self.btn_acc_dist_dec = cp.vl["PCM_BUTTONS"]["BTN_AccDistanceDecrease"]

        # use wheels averages if you like
        # ret.wheelSpeeds = self.get_wheel_speeds(
        #     cp.vl["IPB"]["WheelSpeed_FL"],
        #     cp.vl["IPB"]["WheelSpeed_FR"],
        #     cp.vl["IPB"]["WheelSpeed_RL"],
        #     cp.vl["IPB"]["WheelSpeed_RR"],
        # )
        #speed_kph = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])

        # use dash speedo as speed reference
        speed_raw = int(cp.vl["CARSPEED"]["CarDisplaySpeed"])
        speed_raw_kph = speed_raw * CarControllerParams.K_DASHSPEED
        # Speed calibration from tuning table (was a hardcoded all-1.0 interp = dead code)
        correct_factor = np.interp(speed_raw_kph, Tuning.DASHSPEED_BP, Tuning.DASHSPEED_FP)
        self.speed_kph = speed_raw_kph * correct_factor

        ret.vEgoRaw = float(self.speed_kph * CV.KPH_TO_MS) # KPH to m/s
        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

        ret.yawRate = cp.vl["YAW_RATE"]["YawRate"] - cp.vl["YAW_RATE"]["YawRateOffset"]

        ret.standstill = (speed_raw == 0)

        if self.CP.minSteerSpeed > 0:
            if self.speed_kph > 0.5:
                self.lkas_allowed_speed = True
            elif self.speed_kph < 0.1:
                self.lkas_allowed_speed = False
        else:
            self.lkas_allowed_speed = True

        can_gear = int(cp.vl["DRIVE_STATE"]["Gear"])
        ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

        ret.genericToggle = bool(cp.vl["STALKS"]["HeadLight"])
        if self.CP.enableBsm:
            ret.leftBlindspot = bool(cp.vl["BSD_RADAR"]["LEFT_APPROACH"])
            ret.rightBlindspot = bool(cp.vl["BSD_RADAR"]["RIGHT_APPROACH"])

        ret.leftBlinker = bool(cp.vl["STALKS"]["LeftIndicator"])
        ret.rightBlinker = bool(cp.vl["STALKS"]["RightIndicator"])

        ret.steeringAngleOffsetDeg = 0
        ret.steeringAngleDeg = cp.vl["EPS"]["SteeringAngle"]

        self.steeringRateDegAbs = cp.vl["EPS"]["SteeringAngleRate"]
        ret.steeringRateDeg = self.steeringRateDegAbs

        ret.steeringTorque = cp.vl["ACC_EPS_STATE"]["SteerDriverTorque"]
        ret.steeringTorqueEps = cp.vl["ACC_EPS_STATE"]["MainTorque"]
        #self.eps_warning = bool(cp.vl["ACC_EPS_STATE"]["SteerWarning"]) #Todo: some firmware have SteerWarning field asserted.
        # FIXME(needs real-vehicle validation): reading LKAS_Prepared AND CruiseActivated as a
        # fault is wrong - both are usually set during normal ACC+steering cooperation, which would
        # wrongly trigger steerFaultTemporary while actively controlling. Prefer the raw SteerWarning
        # bit; keeping AccState==7 (ERROR) as the primary fault signal below.
        # 唐DM 实测 (2026-08-30, rlog4 6003帧 100%): ACC_EPS_STATE.SteerWarning 位恒为 1
        # (原车 ESC 固件常态, 非故障), 正常掉头时 |angle| 可达 424 (触发角度检测误判).
        # 故 eps_warning 只读 TorqueFailed (真 EPS 永久故障位, 实测全程 0), 忽略 SteerWarning/角度检测.
        self.eps_warning = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])
        self.eps_state_counter = int(cp.vl["ACC_EPS_STATE"]["Counter"])

        ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > Tuning.STEER_PRESSED_THRESHOLD, 3)

        ret.parkingBrake = (cp.vl["EPB"]["EPB_ActiveFlag"] == 1) if not Tuning.DISABLE_PARKBRAKE else False

        ret.brake =  int(cp.vl["PEDAL"]["BrakePedal"])
        ret.brakePressed = (ret.brake > 5)  # deadzone to avoid noise triggering

        ret.seatbeltUnlatched = (cp.vl["BCM"]["DriverSeatBeltFasten"] != 1) # BCM(0x12D) DriverSeatBeltFasten: 1=fasten, 0=unfasten (对齐 yysnet 验证版位置)

        ret.doorOpen = any([cp.vl["BCM"]["FrontLeftDoor"], cp.vl["BCM"]["FrontRightDoor"],
                            cp.vl["BCM"]["RearLeftDoor"],  cp.vl["BCM"]["RearRightDoor"]])

        ret.gas = int(cp.vl["PEDAL"]["AcceleratorPedal"])
        ret.gasPressed = (ret.gas > 5)  # deadzone to avoid noise triggering

        ret.cruiseState.available = lkas_isMainSwOn and lkas_config_isAccOn and lkas_hud_AccOn1
        ret.cruiseState.enabled = self.acc_state in (3, 5)
        ret.cruiseState.standstill = ret.standstill
        ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SetSpeed"] * CV.KPH_TO_MS
        ret.latEnabled = self.lkas_isMainSwOn and self.lkas_allowed_speed
        #Todo: 唐DM 实测 SteerWarning 恒1 (固件常态), 故 steerFaultTemporary 只由真正的
        # ACC 错误 (acc_state==7, L102-103 已转 0) 触发; eps_warning 已改为 TorqueFailed.
        ret.steerFaultTemporary = bool(self.acc_state == 7)
        #ret.steerFaultTemporary = bool(self.acc_state == 7)

        self.acc_active_last = ret.cruiseState.enabled

        self.mpc_laks_output = cp_cam.vl["ACC_MPC_STATE"]["LKAS_Output"] #use to fool mpc
        self.mpc_laks_reqprepare = cp_cam.vl["ACC_MPC_STATE"]["LKAS_ReqPrepare"] != 0 #use to fool mpc
        self.mpc_laks_active = cp_cam.vl["ACC_MPC_STATE"]["LKAS_Active"] != 0 #use to fool mpc

        self.acc_hud_adas_counter = cp_cam.vl["ACC_HUD_ADAS"]["Counter"]
        self.acc_mpc_state_counter = cp_cam.vl["ACC_MPC_STATE"]["Counter"]
        self.acc_cmd_counter = cp_cam.vl["ACC_CMD"]["Counter"]

        self.cam_lkas = copy.copy(cp_cam.vl["ACC_MPC_STATE"])
        self.cam_acc = copy.copy(cp_cam.vl["ACC_CMD"])
        self.cam_hud = copy.copy(cp_cam.vl["ACC_HUD_ADAS"])  # 原车 ACC_HUD_ADAS 消息，供 create_hud_adas 继承
        self.esc_eps = copy.copy(cp.vl["ACC_EPS_STATE"])

        ret.steerFaultPermanent = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"]) if not Tuning.DISABLE_EPS_PERMANENT_FAULT else False

        # if self.setTimeDelay == 0:
        #     if not system_time_valid():
        #         yyyy = int(cp.vl["DATETIME"]["YY"] + 2000)
        #         MM = int(cp.vl["DATETIME"]["MM"])
        #         DD = int(cp.vl["DATETIME"]["DD"])
        #         hh = int(cp.vl["DATETIME"]["hh"])
        #         mm = int(cp.vl["DATETIME"]["mm"])
        #         ss = int(cp.vl["DATETIME"]["ss"])
        #         china_time = datetime(yyyy,MM,DD,hh,mm,ss)
        #         china_utc_offset = timedelta(hours=8)
        #         utc_time = china_time - china_utc_offset
        #         cloudlog.debug(f"Setting time to {utc_time}")
        #         try:
        #             subprocess.run(f"TZ=UTC date -s '{utc_time}'", shell=True, check=True)
        #         except subprocess.CalledProcessError:
        #             cloudlog.exception("timed.failed_setting_time")
        # else:
        #     self.setTimeDelay = self.setTimeDelay - 1

        ret.buttonEvents = [
            *create_button_events(self.btn_acc_cancel, prev_btn_acc_cancel, {1: ButtonType.cancel}),
            *create_button_events(self.btn_acc_set_reset, prev_btn_acc_set_reset, {1: ButtonType.decelCruise, 3: ButtonType.accelCruise}),
            *create_button_events(self.btn_acc_dist_inc, prev_btn_acc_dist_inc, {1: ButtonType.gapAdjustCruise}),
            *create_button_events(self.btn_acc_dist_dec, prev_btn_acc_dist_dec, {1: ButtonType.gapAdjustCruise}),
        ]

        return ret


    @staticmethod
    def get_can_parsers(CP):
        pt_messages = [
            # sig_address, frequency
            ("EPS", 100),
            ("CARSPEED", 50),
            ("PEDAL", 50),
            ("EPB", 1),
            ("ACC_EPS_STATE", 50),
            ("DRIVE_STATE", 50),
            ("STALKS", 1),
            ("BCM", 1),
            ("PCM_BUTTONS", 20),
            ("DATETIME", 2),
            ("YAW_RATE", 50),
            ("BELT", 20),
        ]

        if CP.enableBsm:
            pt_messages.append(("BSD_RADAR", 20))

        cam_messages = [
            ("ACC_HUD_ADAS", 50),
            ("ACC_CMD", 50),
            ("ACC_MPC_STATE", 50),
        ]
        # 汇报审核 2026-09-04 (纵向优化第一步, 用户批): 让 byd 下发层消费真实前车距离 —
        #   唐DM ARS4xx 雷达主目标 0x109 (=DBC RADAR_MAIN_TARGET) 在 bus1, DBC(byd_han_dmev_2020)
        #   已含 MainDist 信号(0.5*b7-4, 100%验证), 官方机制(CI.update 对 can_parsers.values() 全喂
        #   can_packets, 各 parser 按自己 bus 过滤)天然支持 carstate 挂 bus1 radar parser。
        #   不加第三 parser 时 bus1 雷达帧到车型却不解析 → mrr_leading_dist 无源=199 死值。
        #   (复用与 radar_interface 同源同公式, 非自创读取; DBC/card/acc_cmd 均不动)
        radar_messages = [("RADAR_MAIN_TARGET", 20)]  # 0x109 主目标 MainDist, 20Hz

        return {
            Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.ESC),
            # Intentional: BYD uses a single DBC (dbc_dict has only Bus.pt). The cam messages
            # (ACC_MPC_STATE/ACC_CMD/ACC_HUD_ADAS) live in the same byd_han_dmev_2020.dbc.
            Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus.MPC),
            # 唐DM ARS4xx 雷达在 bus1 (CanBus.MRR=1), DBC 同 byd_han_dmev_2020。radar_interface
            # 也读 bus1(硬编码 CAN_BUS=1), 此处加第三个 parser 供 CS.update 读真实前车距(纵向优化用)。
            Bus.radar: CANParser(DBC[CP.carFingerprint][Bus.pt], radar_messages, CanBus.MRR),
        }
