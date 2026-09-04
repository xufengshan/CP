#!/usr/bin/env python3
from math import exp

from opendbc.car import get_safety_config, get_friction, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, FRICTION_THRESHOLD, LatControlInputs
from opendbc.car.byd.values import CAR, CanBus, BydSafetyFlags, MPC_ACC_CAR
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType
NetworkLocation = structs.CarParams.NetworkLocation

# 唐DM 非线性横向扭矩参数 (实车验证, 对齐 CP11 BydLatUseSiglin=1 默认开)
NON_LINEAR_TORQUE_PARAMS = {
  CAR.BYD_TANG_DM: [1.807, 1.674, 0.04],
}

class CarInterface(CarInterfaceBase):
    CarState = CarState
    CarController = CarController
    RadarInterface = RadarInterface

    def torque_from_lateral_accel_siglin(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                    lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
        friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)

        def sig(val):
            # https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick
            if val >= 0:
                return 1 / (1 + exp(-val)) - 0.5
            else:
                z = exp(val)
                return z / (1 + z) - 0.5

        # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
        non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS[self.CP.carFingerprint]
        a, b, c = non_linear_torque_params
        steer_torque = (sig(latcontrol_inputs.lateral_acceleration * a) * b) + (latcontrol_inputs.lateral_acceleration * c)
        return float(steer_torque) + friction  # 唐DM 非线性扭矩 (干净版, 无除法削弱)

    def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
        # 唐DM 默认 siglin 非线性曲线 (对齐 CP11 BydLatUseSiglin=1)
        return self.torque_from_lateral_accel_siglin

    @staticmethod
    def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, is_release, docs) -> structs.CarParams: # type: ignore
        ret.brand = "byd"
        _safety = structs.CarParams.SafetyModel.byd
        ret.safetyConfigs = [get_safety_config(_safety)]
        # 唐DM safetyParam = HAN_TANG_DMEV(0x1): 默认扭矩 lat 模式 (对齐 CP11 safety_byd.h 默认分支)
        ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.HAN_TANG_DMEV.value

        ret.dashcamOnly = False
        # 唐DM 原厂雷达 = Continental ARS4xx, 由 radar_interface 读 bus1 0x109/0x380 (CP11 已验证控车)
        ret.radarUnavailable = False
        ret.radarTimeStep = 0.04  # 唐DM 主目标 25Hz (1/25; 权威 08-18: 主25.7Hz 副15.3Hz)

        ret.minEnableSpeed = -1.
        ret.enableBsm = 0x418 in fingerprint[CanBus.ESC]
        ret.transmissionType = TransmissionType.direct

        ret.minSteerSpeed = 0.1 * CV.KPH_TO_MS

        ret.steerActuatorDelay = 0.2  # 实车版验证值(对齐加密备份)
        ret.steerLimitTimer = 0.6  # 实车版验证值(对齐加密备份)

        if candidate in MPC_ACC_CAR:
            ret.networkLocation = NetworkLocation.fwdCamera

        # 唐DM 走扭矩 lat 控制 (对齐 CP11)
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

        # 唐DM 直接开纵向 (对齐 CP11 EXP_LONG_CAR)
        ret.alphaLongitudinalAvailable = True
        ret.openpilotLongitudinalControl = True

        ret.longitudinalTuning.kpBP, ret.longitudinalTuning.kiBP = [[0.], [0.]]
        ret.longitudinalTuning.kpV,  ret.longitudinalTuning.kiV  = [[1.0], [0.]]  # kpV=1.0 实车版验证值(对齐加密备份)
        ret.longitudinalTuning.kf = 1.0  # 实车版验证值(对齐加密备份)

        # model specific parameters (唐DM)
        ret.minSteerSpeed = 0
        ret.autoResumeSng = True
        ret.startingState = True
        ret.startAccel = 0.8
        ret.stopAccel = -0.3  # 唐DM 停车减速度 (对齐 CP11)
        ret.vEgoStarting = 0.1 * CV.KPH_TO_MS  # 起步速度阈值 (对齐 CP11)
        ret.vEgoStopping = 0.1 * CV.KPH_TO_MS
        ret.longitudinalActuatorDelay = 0.5

        return ret
