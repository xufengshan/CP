#!/usr/bin/env python3

class Tuning:

  # 以下这组数据，仅力矩模式有效。当开启了LateralTorqueCustom = 1时，以下所有参数都无效。
  LAT_SIGLIN_TABLE = [4.867, 1.09, 0.243] #仅siglin模式有效

  STEERING_ANGLE_OFFSET = 0

  #速度修正参数
  DASHSPEED_BP = [30,   60,   90,  120] #BP是车速
  DASHSPEED_FP = [1.0,  1.0,  1.0, 1.0] #修正百分比

  # modified stock long control 原车long控制的速度平滑百分比设定, 例如下面40米以内，则加速率是原来的70%，减速率是原来的100%
  K_ACCEL_BP       = [40,  50,  60,  70,  80]  # meters BP是离前车距离

  # 以下调校值对齐实车版(加密备份, 已实车验证)
  K_ACCEL_POS_4BAR = [0.8, 0.7, 0.7, 0.7, 0.7] # acceleration 加速的百分比
  K_ACCEL_NEG_4BAR = [1.0, 0.8, 0.7, 0.7, 0.7] # deceleration 减速的百分比

  K_ACCEL_POS_3BAR = [0.8, 0.7, 0.7, 0.7, 0.7] # acceleration 加速的百分比
  K_ACCEL_NEG_3BAR = [1.0, 0.9, 0.8, 0.7, 0.7] # deceleration 减速的百分比

  K_ACCEL_POS_2BAR = [0.8, 0.8, 0.7, 0.7, 0.7] # acceleration 加速的百分比
  K_ACCEL_NEG_2BAR = [1.0, 1.0, 0.9, 0.8, 0.7] # deceleration 减速的百分比

  K_ACCEL_POS_1BAR = [1.0, 1.0, 1.0, 0.9, 0.8] # acceleration 加速的百分比
  K_ACCEL_NEG_1BAR = [1.1, 1.0, 1.0, 1.0, 0.9] # deceleration 减速的百分比

  # 人为扭动方向盘的阈值，大于这个值才认为方向盘被故意扭动了，变道辅助涉及它
  STEER_PRESSED_THRESHOLD = 56

  # 解决某些 D9 或者唐车型，离手时间过久，EPS会退出问题。
  # 解决办法是特定周期退出控制再马上接管（需在 carcontroller 实现消费逻辑，当前仅预留参数）
  HANDSOFF_ANGLE =  [4, 11, 18] #方向盘旋转的角度，不分左右，这里都是正值
  HANDSOFF_PERIOD = [12, 24, 36] #方向盘放开的周期，单位s (2026-09-04 08:49 用户定: HANDSOFF判据改回need_steer后, PERIOD配套回REF[12,24,36], 撤销CP11平均[9,17,26]; index0=12 被 carcontroller 消费, 12s<车机15秒离手退出ACC)

  # EPS 故障/警告计数阈值（需在 carstate 实现消费逻辑，当前仅预留参数）
  EPS_ANGLE_EXCEED_WARNING_CNT = 3
  EPS_ANGLE_SPEED_WARNING_CNT = 3

  # 禁用EPS故障检查, 某些车有EPS固件比较奇怪报错的话，则可以设为True
  DISABLE_EPS_WARNING = False
  DISABLE_EPS_TEMPORARY_FAULT = False
  DISABLE_EPS_PERMANENT_FAULT = True

  DISABLE_PARKBRAKE = False
