import numpy as np
from opendbc.car import structs
from opendbc.car.byd.values import  CanBus, CarControllerParams

GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def byd_checksum(byte_key, dat):
    first_bytes_sum = sum(byte >> 4 for byte in dat)
    second_bytes_sum = sum(byte & 0xF for byte in dat)
    remainder = second_bytes_sum >> 4
    second_bytes_sum += byte_key >> 4
    first_bytes_sum += byte_key & 0xF
    first_part = ((-first_bytes_sum + 0x9) & 0xF)
    second_part = ((-second_bytes_sum + 0x9) & 0xF)
    return (((first_part + (-remainder + 5)) << 4) + second_part) & 0xFF

# MPC -> Panda -> EPS
def create_steering_control(packer, CP, cam_msg: dict, req_torque, req_prepare, active, hud_control, counter):
    values = {}
    values = {s: cam_msg[s] for s in [
        "AutoFullBeamState",
        "LeftLaneState",
        "LKAS_Config",
        "SETME2_0x1",
        "MPC_State",
        "AutoFullBeam_OnOff",
        "LKAS_Output",
        "LKAS_Active",
        "SETME3_0x0",
        "TrafficSignRecognition_OnOff",
        "SETME4_0x0",
        "SETME5_0x1",
        "RightLaneState",
        "LKAS_State",
        "TrafficSignRecognition_Result",
        "LKAS_AlarmType",
        "SETME7_0x3",
    ]}

    values["ReqHandsOnSteeringWheel"] = 0
    values["LKAS_ReqPrepare"] = req_prepare
    values["Counter"] = counter

    if active:
        mpc_state = values["MPC_State"] #2: Cancelling lkas control
        values.update({
            "LKAS_Output" : req_torque,
            "LKAS_Active" : 1,
            "LKAS_State" : 4 if (mpc_state == 2) else 2,
            "LeftLaneState":  3 if hud_control.leftLaneDepart  else int(hud_control.leftLaneVisible) + 1,
            "RightLaneState": 3 if hud_control.rightLaneDepart else int(hud_control.rightLaneVisible) + 1,
        })
    else: # Note: This disables the stock AEB feature: turn steering wheel while close impacting obstacles in front.
        values.update({
            "LKAS_Output" : 0,
            "LKAS_Active" : 0,
        })

    # 🔧 2026-09-01 18:58 移除 b0/b1/b5 强制覆盖 (组合方案 — 用户定), 回退 V9 纯标准 packer
    #   根因: 我们强制 b0=E2/C2 + b1=81 + b5=11 覆盖 cam_msg 原车视频器字节, 制造按键/限速状态冲突
    #         → 上下键不稳定 + 限速标记常亮
    #   修复: 对齐 V9 (横向成功版) — 纯 packer 继承 cam_msg, 无手工字节覆盖, CheckSum 用全 data
    data = packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)

# op long control
def acc_cmd(packer, CP, cam_msg: dict, mrr_leaddist, accel, rfss, sss, longActive):
    values = {}

    values = {s: cam_msg[s] for s in [
        "AccelCmd",
        "ComfortBandUpper",
        "ComfortBandLower",
        "JerkUpperLimit",
        "SETME1_0x1",
        "JerkLowerLimit",
        "ResumeFromStandstill",
        "StandstillState",
        "BrakeBehaviour",
        "AccReqNotStandstill",
        "AccControlActive",
        "AccOverrideOrStandstill",
        "EspBehaviour",
        "Counter",
        "SETME2_0xF",
    ]}

    jerk_base_upper = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_upper_fp)
    jerk_base_lower = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_lower_fp)

    if (accel < 0): #use lower factor
        jerk_upper = jerk_base_upper
        jerk_lower = jerk_base_lower + accel * CarControllerParams.K_accel_jerk_lower
    else:
        jerk_upper = jerk_base_upper + accel * CarControllerParams.K_accel_jerk_upper
        jerk_lower = jerk_base_lower

    # 门(mrr_leaddist>3) = 无雷达/雷达故障时的安全保证(保留 REF 原逻辑, 不能删):
    #   雷达正常时 mrr=真实前车距(>3 才发, 近距离保护); 无雷达/故障时 mrr 回 199 兜底(恒>3 恒发, 纵向不丢)。
    #   真实 mrr 仅在此用作"最小距离保护门", 不替代上层 OP/原车对 jerk/ComfortBand 的裁决。
    #   (08:21 用户定: 门本身没问题必须留; 引起自己加速是我此前在门上叠真实距离查表改写控制的新逻辑, 已去除—
    #    查表/ComfortBand 恢复 REF 原样, 只把 mrr 喂真实值+无雷达199兜底, 不额外加任何自创改写)
    if longActive and mrr_leaddist > 3:  # 增加最小距离检测
        values.update({
            "AccelCmd" : accel,
            "ComfortBandUpper" : 0.05 if mrr_leaddist > 50 else 0.10,
            "ComfortBandLower" : 0.05 if mrr_leaddist > 50 else 0.10,
            "JerkUpperLimit" : jerk_upper,
            "JerkLowerLimit" : jerk_lower,
            "ResumeFromStandstill" : rfss,
            "StandstillState" : sss,
        })

    data = packer.make_can_msg("ACC_CMD", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data[:7] + b'\x00')
    return packer.make_can_msg("ACC_CMD", CanBus.ESC, values)


# send fake torque feedback from eps to trick MPC, preventing DTC, so that safety features such as AEB still working
def create_fake_318(packer, CP, esc_msg: dict, faketorque, laks_reqprepare, laks_active , enabled, counter):
    values = {}

    # 真机 DBC = byd_han_dmev_2020.dbc (无 LKAS_State 枚举, 用 LKAS_Prepared + CruiseActivated 旧字段)
    # V9 用 byd_tang_dm.dbc 才有 LKAS_State 2bit 枚举 — 但真机 DBC 不同, 不能用枚举 (会 KeyError)
    values = {s: esc_msg[s] for s in [
        "LKAS_State",
        "TorqueFailed",
        "SETME1_0x1",
        "SteerWarning",
        "SteerErrorCode",
        "MainTorque",
        "SETME3_0x1",
        "SETME4_0x3",
        "SteerDriverTorque",
        "SETME5_0xFF",
        "SETME6_0xFFF",
    ]}

    values["ReportHandsNotOnSteeringWheel"] = 0
    values["Counter"] = counter

    # 🔧 2026-09-01 18:58 移除 b5=FF 强制覆盖 (组合方案 — 用户定), 对齐 V9 CheckSum 用全 data
    #   (b5=FF 强制覆盖继承字节制造状态错乱; 回退让 esc_msg 自然继承 + CheckSum 全 data)
    if enabled:
        if laks_active:
            values.update({
                "LKAS_State" : 2,  # Active
                "MainTorque" : int(faketorque),
            })
        elif laks_reqprepare:
            values.update({
                "LKAS_State" : 1,  # Ready
                "MainTorque" : 0,
            })
        else:
            values.update({
                "LKAS_State" : 0,  # NotReady
                "MainTorque" : 0,
            })

    data = packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)


# ============================================================================
# ACC_HUD_ADAS (0x32D) — 原车 ACC 视觉状态广播帧
# ============================================================================
# CP 替代原车视觉后，openpilot 纵向接管时必须自己发 ACC_HUD_ADAS 到 ESC bus，
# 诱骗原车 ACC 控制单元认为"ACC 已装备、链路正常"。
#
# 参考 CP 作者使用的现代 (Hyundai) 方法 (carcontroller.py):
#   - create_acc_opt (SCC13):   frame%20==0 且 openpilotLongitudinalControl 时发
#                               SCC_Equip=1 告诉原车"ACC 已装备"
#   - create_frt_radar_opt:     frame%50==0 时发 CF_FCA_Equip_Front_Radar=1
#                              告诉原车"前置雷达已装备"
#   - make_tester_present:      frame%100==0 时发 0x7d0 禁用原厂雷达/ADAS ECU
#
# BYD 对应: ACC_HUD_ADAS 广播 "ACC 状态/装备/前车/HUD" 给原车，
#           AccState 必须为健康值(0/3)，绝不广播 7(ERROR) 否则原车报"雷达错误"。
#
# 信号 (byd_han_dmev_2020.dbc BO_ 813): SetSpeed 0|9, HasLead 9|1,
#   SetDistance 10|3, LeadingDistance 13|3, AEB 16|1, FCW 17|1, SETME1 18|1,
#   AccState 19|3 (0=OFF 2=ON 3=ACTIVE 5=FORCE 7=ERROR), AccOn1 22|1,
#   CloseWarning 23|1, SETME2 24|1, Notify 25|7, Status 32|4,
#   SETME3 36|12, Counter 48|4, SETME4 55|4, CheckSum 56|8
def create_hud_adas(packer, CP, cam_hud: dict, CS, CC, longActive, counter):
    values = {}

    # 继承原车摄像头能读到的信号（保留大部分原车状态，只覆盖 ACC 健康状态）
    # 参考丰田 create_ui_command "if len(stock_lkas_hud): update(collected)" 模式
    if cam_hud is not None and len(cam_hud) > 0:
        values = {s: cam_hud[s] for s in [
            "SetSpeed",
            "HasLead",
            "SetDistance",
            "LeadingDistance",
            "AEB",
            "FCW",
            "SETME1_0x1",
            "AccOn1",
            "CloseWarning",
            "SETME2_0x1",
            "Status",
            "SETME3_0xFFF",
            "SETME4_0xF",
        ]}
    else:
        # 无原车摄像头消息时用安全默认值（永不广播 ERROR）
        values = {
            "SetSpeed": 0,
            "HasLead": 0,
            "SetDistance": 0,
            "LeadingDistance": 0,
            "AEB": 0,
            "FCW": 0,
            "SETME1_0x1": 1,
            "AccOn1": 0,
            "CloseWarning": 0,
            "SETME2_0x1": 1,
            "Notify": 0,          # 0 = NONE（不广播 ACC_ERROR）
            "Status": 0,
            "SETME3_0xFFF": 4095,
            "SETME4_0xF": 3,
        }

    # 只覆盖 ACC 健康接管状态 (AccState) + 主开关 (AccOn1) + Counter + CheckSum
    # 其余信号 (SetSpeed/SetDistance/Status/Notify/LeadingDistance/SETME4 等)
    # 100% 继承 cam_hud 原车摄像头值, 绝不硬编码覆盖!
    # (铁证: 原车 src2 实时帧 SetSpeed=30/Status=4/Notify=0/SETME4=7,
    #  硬编码覆盖会把它们改成 60/3/8/9 → 原车ACC判定链路异常 → "自动制动功能受限")
    # AccState 状态机(对齐路试黄金帧): 3=ACTIVE(ENGAGED) / 2=READY(主开关开未engage) / 0=OFF
    # 绝不广播 7(ERROR) — 对齐黄金 ENGAGED=3 READY=2
    acc_active = bool(longActive and (CC.enabled if CC is not None else False))
    main_sw = getattr(CS, 'lkas_isMainSwOn', None) if CS is not None else None
    if acc_active:
        values["AccState"] = 3   # ACC_ACTIVE (ENGAGED) — 对齐黄金
    elif main_sw:
        values["AccState"] = 2   # ACC_READY (对齐CP11路试黄金标: READY=2, ACTIVE=3, OFF=0) (主开关开, 未engage) — 对齐黄金 READY
    else:
        values["AccState"] = 0   # OFF
    # AccOn1 = 原车 ACC 主开关状态 (PCM_BUTTONS.BTN_TOGGLE_ACC_OnOff)
    # 关键: cruiseState.available = lkas_isMainSwOn and lkas_config_isAccOn and AccOn1
    # 若 AccOn1 只在激活(AccState!=0)时=1, 会导致 available=False → wrongCarMode → 无法engage (死锁)
    # 所以 AccOn1 必须跟随原车主开关, 用户按ACC主开关后即可engage
    main_sw = getattr(CS, 'lkas_isMainSwOn', None) if CS is not None else None
    if main_sw is not None:
        values["AccOn1"] = 1 if main_sw else 0
    else:
        # 无 CS 时保持 AccState 关联（兼容测试桩）
        values["AccOn1"] = 1 if (values["AccState"] != 0) else int(values.get("AccOn1", 0))
    # Notify 强制 = 0 (对齐黄金 8/14: 视频器/OP 0x32D b3=01 恒定, Notify=0)
    # 原车视频器异常时可能广播 Notify=16 (b3=21), 若 OP 继承会导致 b3=21 偏离黄金 → 原车ACC判链路异常
    values["Notify"] = 0
    values["Counter"] = counter

    data = packer.make_can_msg("ACC_HUD_ADAS", CanBus.ESC, values)[1]
    # ⭐ 黄金恒定字节对齐 (8/14 全 session): 0x32D b4=F4, b5=FF 恒不变 (所有状态) — 强制, 不依赖视频器继承
    data = data[:4] + bytes([0xF4, 0xFF]) + data[6:]
    values["CheckSum"] = byd_checksum(0xAF, data[:7] + b'\x00')
    out = packer.make_can_msg("ACC_HUD_ADAS", CanBus.ESC, values)
    # make_can_msg 会用 values 信号重编码覆盖 b4/b5 → 最后手动强制黄金恒定字节 + 重算 CheckSum
    addr, d, bus = out
    d = d[:4] + bytes([0xF4, 0xFF]) + d[6:]
    cs = byd_checksum(0xAF, d[:7] + b'\x00')
    return (addr, d[:7] + bytes([cs]), bus)

# ⭐ 补发 0x3B0 ACC_PCM_BUTTONS (2026-09-01 黄金铁证: 黄金 src=0 发此帧 1201帧/60s≈20Hz)
#   黄金帧: 04 11 00 00 00 00 [counter<<4] [checksum]  — BTN_TOGGLE_ACC_OnOff=1 (ACC主开关常开)
#   我们此前完全没发 0x3B0 → 原车ACC主开关链路缺失 → 横向激活时报"限速/多功能视频器"!
#   补发对齐黄金 → 消除报警 (连带: 纵向横向都依赖ACC主开关)
def create_pcm_buttons(packer, counter):
    values = {
        "SETME_1": 1,
        "BTN_AccUpDown_Cmd": 0,
        "BTN_AccCancel": 0,
        "BTN_TOGGLE_ACC_OnOff": 1,   # ACC 主开关常开 (对齐黄金)
        "SETME2_1": 1,
        "BTN_AccDistanceDecrease": 0,
        "BTN_AccDistanceIncrease": 0,
        "Counter": counter,
    }
    data = packer.make_can_msg("PCM_BUTTONS", CanBus.MPC, values)[1]
    # 黄金恒定: b0=04 b1=11 b2-5=00, b6=counter高4位, b7=checksum
    data = data[:6] + bytes([(counter & 0xF) << 4]) + data[7:]
    cs = byd_checksum(0xAF, data[:7] + b'\x00')
    return (0x3B0, data[:7] + bytes([cs]), CanBus.MPC)

