from dataclasses import dataclass, field
from enum import IntFlag
from opendbc.car import Bus, DbcDict, PlatformConfig, Platforms, CarSpecs
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  # 2026-09-04 09:28 用户定: 恢复横向参数为 REF(正常版 C:\Users\xufen\Desktop\opendbc) 值 —— 路试横向不稳定。
  #   REF 长时间稳定跑过, 300/12 改动(09-03/09-04)后横向不稳定 → 全部回 REF 280/6/9。
  #   (此前曾因 EPS实测-297~316/tight弯建矩 调大过; 现据路试回归 REF 保守值优先稳定性)
  STEER_MAX = 280   # REF (原曾 290→300, 现回 280)
  STEER_DELTA_UP = 6   # REF (原曾 16→12, 现回 6)
  STEER_DELTA_DOWN = 9

  # (2026-09-02 H3 曾提议 68→110, 已撤回: yysnet 汉官方验证版即用 68 无对抗问题;
  #  黄金"司机单打手力大 -316"不适用"司机+OP对抗"场景; 68 让司机抢盘更早更安全. 保持官方 68)
  STEER_DRIVER_ALLOWANCE = 68
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50

  STEER_STEP = 2  #100/2=50hz
  STEER_SOFTSTART_STEP = 6 # 20ms(50Hz) * 300 / 6 = 1000ms. This means the clip ceiling will be increased to 300 in 1000ms

  # BYD EPS 在车近乎静止(vEgo<STEER_LOW_SPEED_V)时最大可承受电机扭矩约60,
  # 超过即永久 TorqueFailed(需重启车)。此处仅在 vEgo<0.6 m/s 时把最终下发扭矩限到 ±STEER_LOW_SPEED_MAX(40),
  # 对应最坏eps~45(留~15余量)。hands-off 时 eps≈cmd(1:1, 实测EPS放大≤13%)。
  # (V9 横向 100% 学习: 2026-09-02 补, 防 ESC TorqueFailed 永久故障)
  STEER_LOW_SPEED_V = 0.6      # m/s, 进入静止保护的车速阈值
  STEER_LOW_SPEED_MAX = 40     # raw 扭矩单位(= motor torque), 对应最坏eps~45(留~15余量)

  ACC_STEP = 2    #50hz

  ACCEL_MAX = 2.0
  ACCEL_MIN = -3.5

  K_DASHSPEED = 0.0719088 #convert pulse to kph

  USE_STEERING_SPEED_LIMITER = False

  # op long control
  K_accel_jerk_upper = 0.1
  K_accel_jerk_lower = 0.5
  K_jerk_xp =            [   4,   10,   20,   40,   80]  # meters
  # 汇报审核 2026-09-03 (用户定选B最小改): 近距 4m 端 -2.3→-1.8 收浅。
  #   动机: realdata 全量 506 次自主无脚刹车事件长尾过猛(8次>0.35g/14次>0.3g/最深aEgo-5.23), 集中在近距离前车收紧;
  #         现 acc_cmd 刹车时 jerk_lower = base + accel*0.5, 4m端-2.3 + (-3.5*0.5) 达 -4.05, 近距第一口太陡。
  #   改后 4m端: accel-3.5 → -3.55(较-4.05收浅~0.5), 渐进起步刹车; 仍保留刹到-3.5能力。ACCEL_MIN=-3.5/K_jerk_xp/中远距不动。
  #   (2026-09-03 用户批AB执行): 方案A铺开已完成 → 10m:-1.8→-1.5 / 20m:-1.4→-1.3, 近距整段缓刹; 4m保持-1.8/40m-1.0/80m-0.4。
  K_jerk_base_lower_fp = [-1.8, -1.5, -1.3, -1.0, -0.4]
  K_jerk_base_upper_fp = [ 0.8,  0.7,  0.6,  0.3,  0.2]

  def __init__(self, CP):
    pass

#FD to be added later
class BydSafetyFlags(IntFlag):
  HAN_TANG_DMEV = 0x1 #pre 2021 models with veoneer mpc/radar solution
  TANG_DMI = 0x2 #note tang dmi is not tang dm
  SONG_PLUS_DMI = 0x4 #note song pro is similar but not song dmi
  QIN_PLUS_DMI = 0x8
  YUAN_PLUS_DMI_ATTO3 = 0x10 #yuan plus is atto3
  ACC_CRUISEDISP = 0x20  # 实车版: 唐DMp22/汉DMI22R/腾势D9_22R 用 cruise display 加速响应
  ANGLE_MODE = 0x40  # 实车版: 海豹/腾势用角度控制模式(非扭矩)
  ACC_ON1 = 0x80  # 实车版: 海豹用 ACC 信号差异


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  #todo add docs and harness info

@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "byd_han_dmev_2020"})
  #todo add dbc for other models

class CAR(Platforms):
  BYD_TANG_DM = BydPlatformConfig(
    [BydCarDocs("BYD TANG DM")],
    CarSpecs(mass=2390., wheelbase=2.820, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

class LKASConfig:
  DISABLE = 0
  ALARM = 1
  LKA = 2
  ALARM_AND_LKA = 3

class CanBus:
  ESC = 0
  MRR = 1
  MPC = 2

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=CanBus.ESC,
    ),
  ],
)

# ==== 唐DM 单车规范版 (carrot-wip 9081364): 平台/总线/控制断言 仅唐DM ====
MPC_ACC_CAR = {CAR.BYD_TANG_DM}    # power train canbus 位于 MPC 连接器
PT_RADAR_CAR = {CAR.BYD_TANG_DM}   # power train canbus 含 mrr 雷达信息
TORQUE_LAT_CAR = {CAR.BYD_TANG_DM} # 唐DM 扭矩横向控制
EXP_LONG_CAR = {CAR.BYD_TANG_DM}   # 唐DM experimental long

DBC = CAR.create_dbc_map()

if __name__ == "__main__":
  cars = []
  for platform in CAR:
    for doc in platform.config.car_docs:
      cars.append(doc.name)
  cars.sort()
  for c in cars:
    print(c)
