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
from opendbc.car.byd.values import DBC, CanBus, LKASConfig, CarControllerParams
from opendbc.car.byd.tuning import Tuning

import os
BYD_RADAR = os.getenv("BYD_RADAR") is not None

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
        self.acc_state = 0
        self.adas_set_dist = 0

        self.mpc_laks_output = 0
        self.mpc_laks_active = False
        self.mpc_laks_reqprepare = False

        #self.prev_angle = 0

        self.cam_lkas = 0
        self.cam_acc = 0
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

        self.lkas_prepared = cp.vl["ACC_EPS_STATE"]["LKAS_Prepared"]
        self.esp_lkas_CruiseActivated = cp.vl["ACC_EPS_STATE"]["CruiseActivated"]

        self.mpc_lkas_config = int(cp_cam.vl["ACC_MPC_STATE"]["LKAS_Config"])
        lkas_config_isAccOn = (self.mpc_lkas_config != LKASConfig.DISABLE)
        lkas_isMainSwOn = bool(cp.vl["PCM_BUTTONS"]["BTN_TOGGLE_ACC_OnOff"])
        self.lkas_isMainSwOn = bool(cp.vl["PCM_BUTTONS"]["BTN_TOGGLE_ACC_OnOff"])
        lkas_hud_AccOn1 = bool(cp_cam.vl["ACC_HUD_ADAS"]["AccOn1"])
        self.acc_state  = cp_cam.vl["ACC_HUD_ADAS"]["AccState"]
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
        # EPS angle/rate abnormal detection (need consecutive exceed to avoid false positive)
        if abs(ret.steeringAngleDeg) > 400.0 or abs(self.steeringRateDegAbs) > 100.0:
            self.eps_angle_exceed_cnt += 1
            self.eps_angle_speed_cnt += 1
        else:
            self.eps_angle_exceed_cnt = 0
            self.eps_angle_speed_cnt = 0

        extra_warning = (self.eps_angle_exceed_cnt >= Tuning.EPS_ANGLE_EXCEED_WARNING_CNT or
                         self.eps_angle_speed_cnt >= Tuning.EPS_ANGLE_SPEED_WARNING_CNT)
        self.eps_warning = (bool(cp.vl["ACC_EPS_STATE"]["SteerWarning"]) or extra_warning) if not Tuning.DISABLE_EPS_WARNING else False
        self.eps_state_counter = int(cp.vl["ACC_EPS_STATE"]["Counter"])

        ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > Tuning.STEER_PRESSED_THRESHOLD, 3)

        ret.parkingBrake = (cp.vl["EPB"]["EPB_ActiveFlag"] == 1) if not Tuning.DISABLE_PARKBRAKE else False

        ret.brake =  int(cp.vl["PEDAL"]["BrakePedal"])
        ret.brakePressed = (ret.brake > 5)  # deadzone to avoid noise triggering

        ret.seatbeltUnlatched = (cp.vl["BELT"]["SeatBeat"] != 2) # 1:unfasten, 2:fasten

        ret.doorOpen = any([cp.vl["BCM"]["FrontLeftDoor"], cp.vl["BCM"]["FrontRightDoor"],
                            cp.vl["BCM"]["RearLeftDoor"],  cp.vl["BCM"]["RearRightDoor"]])

        ret.gas = int(cp.vl["PEDAL"]["AcceleratorPedal"])
        ret.gasPressed = (ret.gas > 5)  # deadzone to avoid noise triggering

        ret.cruiseState.available = lkas_isMainSwOn and lkas_config_isAccOn and lkas_hud_AccOn1
        ret.cruiseState.enabled = self.acc_state in (3, 5)
        ret.cruiseState.standstill = ret.standstill
        ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SetSpeed"] * CV.KPH_TO_MS
        ret.latEnabled = self.lkas_isMainSwOn and self.lkas_allowed_speed
        #Todo: some firmware have these fields asserted.
        ret.steerFaultTemporary = bool((self.acc_state == 7) or self.eps_warning) if not Tuning.DISABLE_EPS_TEMPORARY_FAULT else False
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
        self.esc_eps = copy.copy(cp.vl["ACC_EPS_STATE"])

        if BYD_RADAR:
            mrr_id = int(cp_cam.vl["RADAR_MRR"]["TargetID"])

            if mrr_id == 2: #1:left, 2:front, 3:right
                if bool(cp_cam.vl["RADAR_MRR"]["IsValid"]):
                    raw_dist = int(cp_cam.vl["RADAR_MRR"]["LongDist"])
                    # 增加距离滤波，避免异常值导致误判
                    if 3 < raw_dist < 200:
                        self.mrr_leading_dist = raw_dist
                    else:
                        self.mrr_leading_dist = 199  # 无效/越界距离回兜底
                else:
                    self.mrr_leading_dist = 199
            else:
                self.mrr_leading_dist = 199  # 非前向目标(左/右)无前车距离, 回兜底避免 stale

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
        if BYD_RADAR:
            cam_messages.append(("RADAR_MRR", 60))

        return {
            Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.ESC),
            # Intentional: BYD uses a single DBC (dbc_dict has only Bus.pt). The cam messages
            # (ACC_MPC_STATE/ACC_CMD/ACC_HUD_ADAS) live in the same byd_han_dmev_2020.dbc.
            Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus.MPC),
        }
