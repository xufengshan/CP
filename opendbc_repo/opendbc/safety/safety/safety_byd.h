#pragma once

#include "safety_declarations.h"

//Todo: Support of other platforms to be verified and added later, placeholders for now
//Currently been tested on: Han DM and EV 21 models

#define BYD_CANADDR_IPB               0x1F0
#define BYD_CANADDR_ACC_MPC_STATE     0x316
#define BYD_CANADDR_ACC_EPS_STATE     0x318
#define BYD_CANADDR_ACC_HUD_ADAS      0x32D
#define BYD_CANADDR_ACC_CMD           0x32E
#define BYD_CANADDR_PCM_BUTTONS       0x3B0
#define BYD_CANADDR_DRIVE_STATE       0x242
#define BYD_CANADDR_PEDAL             0x342
#define BYD_CANADDR_CARSPEED          0x121

#define BYD_CANBUS_ESC  0
#define BYD_CANBUS_MRR  1
#define BYD_CANBUS_MPC  2


static bool byd_eps_cruiseactivated = false;

// typedef enum {
//   HAN_TANG_DMEV,
//   TANG_DMI,
//   SONG_PLUS_DMI,
//   QIN_PLUS_DMI,
//   YUAN_PLUS_DMI_ATTO3
// } BydPlatform;
// static BydPlatform byd_platform;

static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);
  if (bus == BYD_CANBUS_ESC) {
    if (addr == BYD_CANADDR_PEDAL) {
      gas_pressed = (GET_BYTE(to_push, 0) != 0U);
      brake_pressed = (GET_BYTE(to_push, 1) != 0U);
    } else if (addr == BYD_CANADDR_CARSPEED) {
      unsigned int speed_dash_LSB = GET_BYTE(to_push, 0);
      unsigned int speed_dash_MSB = GET_BYTE(to_push, 1) & 0xFU;
      vehicle_moving = (speed_dash_LSB > 0U) || (speed_dash_MSB > 0U);
    } else if (addr == BYD_CANADDR_ACC_EPS_STATE) {
      byd_eps_cruiseactivated = (GET_BYTE(to_push, 0) & 0x3U) == 2U; // LKAS_State Active (byte0低2位==2)
      int torque_motor = ((GET_BYTE(to_push, 2) & 0xFU) << 8) | GET_BYTE(to_push, 1); // MainTorque
      torque_motor = to_signed(torque_motor, 12);
      update_sample(&torque_meas, torque_motor);
    }
    else {
      //empty
    }
  } else if (bus == BYD_CANBUS_MPC) {
    if (addr == BYD_CANADDR_ACC_HUD_ADAS) {
      unsigned int accstate = ((GET_BYTE(to_push, 2) >> 3) & 0x07U);
      bool cruise_engaged = (accstate == 3U) || (accstate == 5U); // 3=acc_active, 5=user force accel
      pcm_cruise_check(cruise_engaged);
    }
  }
  else {
    //empty
  }
}


static bool byd_tx_hook(const CANPacket_t *to_send) {
  const TorqueSteeringLimits HAN_DMEV_STEERING_LIMITS = {
    .max_steer = 300,
    .max_rate_up = 18,
    .max_rate_down = 18,
    .max_rt_delta = 250,
    .max_rt_interval = 250000,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };
  // const TorqueSteeringLimits TANG_DMI_STEERING_LIMITS = { //values to be check
  //   .max_torque = 300,
  //   .max_rate_up = 9,
  //   .max_rate_down = 9,
  //   .max_rt_delta = 113,
  //   .max_torque_error = 80,
  //   .type = TorqueMotorLimited,
  // };
  // const TorqueSteeringLimits SONG_STEERING_LIMITS = { //values to be check
  //   .max_torque = 300,
  //   .max_rate_up = 9,
  //   .max_rate_down = 9,
  //   .max_rt_delta = 113,
  //   .max_torque_error = 80,
  //   .type = TorqueMotorLimited,
  // };
  // const TorqueSteeringLimits QIN_STEERING_LIMITS = { //values to be check
  //   .max_torque = 300,
  //   .max_rate_up = 9,
  //   .max_rate_down = 9,
  //   .max_rt_delta = 113,
  //   .max_torque_error = 80,
  //   .type = TorqueMotorLimited,
  // };
  // const TorqueSteeringLimits YUAN_ATTO3_STEERING_LIMITS = { //values to be check
  //   .max_torque = 300,
  //   .max_rate_up = 9,
  //   .max_rate_down = 9,
  //   .max_rt_delta = 113,
  //   .max_torque_error = 80,
  //   .type = TorqueMotorLimited,
  // };

  bool tx = true;
  int bus = GET_BUS(to_send);

  if (bus == BYD_CANBUS_ESC) {
    int addr = GET_ADDR(to_send);
    if (addr == BYD_CANADDR_ACC_MPC_STATE) {
      int desired_torque = ((GET_BYTE(to_send, 3) & 0x07U) << 8U) | GET_BYTE(to_send, 2);
      desired_torque = to_signed(desired_torque, 11);
      bool steer_req = GET_BIT(to_send, 28U) || byd_eps_cruiseactivated; //LKAS_Active

      // const TorqueSteeringLimits limits = (byd_platform == HAN_TANG_DMEV) ? HAN_DMEV_STEERING_LIMITS :
      //                                     (byd_platform == TANG_DMI) ? TANG_DMI_STEERING_LIMITS :
      //                                     (byd_platform == SONG_PLUS_DMI) ? SONG_STEERING_LIMITS :
      //                                     (byd_platform == QIN_PLUS_DMI) ? QIN_STEERING_LIMITS : YUAN_ATTO3_STEERING_LIMITS;

      const TorqueSteeringLimits limits = HAN_DMEV_STEERING_LIMITS;
      if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
        tx = true;  // 放行 OP 横向扭矩(0x316): 通过安全校验(限幅/速率/rt)即放行 (2026-09-02 修正, 原误 tx=false 拦截横向致车不拐)
      }
    }

  }

  return tx;
}

static int byd_fwd_hook(CANPacket_t* to_send) {
  const int bus_num = GET_BUS(to_send);
  const int addr = GET_ADDR(to_send);

  // 黄金路试 fwd_hook 语义 (8/14 实证, 与 yysnet 汉验证版一致): 
  //   - ESC(0) -> MPC(2): 转发所有, 但 block 0x318 (ACC_EPS_STATE, OP 自己伪造发 bus2)
  //   - MPC(2) -> ESC(0): 转发所有, 但 block 0x316 (ACC_MPC_STATE) + 0x32E (ACC_CMD) (OP 自己控制)
  //     ⚠️ 不 block 0x32D (ACC_HUD_ADAS)! 原车摄像头 0x32D 必须转发到 ESC:
  //     ESC 需要看到原车 ACC 状态(0x32D AccState) 才执行横向 + 不报"自动制动功能受限"
  //     之前错误新增 block 0x32D(0710d0ab) → ESC 收不到原车 0x32D → ACC 状态异常 → 拒绝执行 OP 横向!
  int bus_fwd = -1;

  if (bus_num == BYD_CANBUS_ESC) {
    const bool is_eps = (addr == BYD_CANADDR_ACC_EPS_STATE);
    if (!is_eps) {
      bus_fwd = BYD_CANBUS_MPC;
    }
  } else if (bus_num == BYD_CANBUS_MPC) {
    const bool is_lkas = ((addr == BYD_CANADDR_ACC_MPC_STATE) || (addr == BYD_CANADDR_ACC_CMD));
    if (!is_lkas) {
      bus_fwd = BYD_CANBUS_ESC;
    }
  }

  return bus_fwd;
}

static safety_config byd_init(uint16_t param) {
  UNUSED(param);
  // const uint32_t FLAG_HAN_TANG_DMEV = 0x1U;
  // const uint32_t FLAG_TANG_DMI = 0x2U;
  // const uint32_t FLAG_SONG_PLUS_DMI = 0x4U;
  // const uint32_t FLAG_QIN_PLUS_DMI = 0x8U;
  // const uint32_t FLAG_YUAN_PLUS_DMI_ATTO3 = 0x10U;

  static const CanMsg BYD_HAN_DMEV_TX_MSGS[] = {
    {BYD_CANADDR_ACC_CMD,         BYD_CANBUS_ESC, 8},
    {BYD_CANADDR_ACC_HUD_ADAS,    BYD_CANBUS_ESC, 8},  // OP 发健康帧安抚原车 ACC
    {BYD_CANADDR_ACC_MPC_STATE,   BYD_CANBUS_ESC, 8},
    {BYD_CANADDR_ACC_EPS_STATE,   BYD_CANBUS_MPC, 8},
    {BYD_CANADDR_PCM_BUTTONS,     BYD_CANBUS_MPC, 8},  // ⭐ 0x3B0 ACC主开关(09-01 黄金实证), OP 发 MPC bus
  };

  // static const CanMsg BYD_YUANPLUS_ATTO3_TX_MSGS[] = {
  //   {BYD_CANADDR_ACC_CMD,         BYD_CANBUS_ESC, 8},
  //   {BYD_CANADDR_ACC_MPC_STATE,   BYD_CANBUS_ESC, 8},
  //   {BYD_CANADDR_ACC_EPS_STATE,   BYD_CANBUS_MPC, 8},
  // };

  static RxCheck byd_han_dmev_rx_checks[] = {
    {.msg = {{BYD_CANADDR_PEDAL,         BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_CARSPEED,      BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_EPS_STATE, BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_HUD_ADAS,  BYD_CANBUS_MPC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  };

  //   static RxCheck byd_yuanplus_atto3_rx_checks[] = {
  //     {.msg = {{BYD_CANADDR_PEDAL,         BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  //     {.msg = {{BYD_CANADDR_CARSPEED,      BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  //     {.msg = {{BYD_CANADDR_ACC_EPS_STATE, BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  //     {.msg = {{BYD_CANADDR_ACC_HUD_ADAS,  BYD_CANBUS_MPC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  //   };

  safety_config ret;

  // bool use_han_dm = GET_FLAG(param, FLAG_HAN_TANG_DMEV);
  // bool use_tang_dmi = GET_FLAG(param, FLAG_TANG_DMI);
  // bool use_song = GET_FLAG(param, FLAG_SONG_PLUS_DMI);
  // bool use_qin = GET_FLAG(param, FLAG_QIN_PLUS_DMI);
  // bool use_yuan = GET_FLAG(param, FLAG_YUAN_PLUS_DMI_ATTO3);

  // if (use_tang_dmi || use_song || use_qin) {
  //   byd_platform = TANG_DMI;
  //   ret = BUILD_SAFETY_CFG(byd_han_dmev_rx_checks, BYD_HAN_DMEV_TX_MSGS);
  // } else if (use_yuan) {
  //   byd_platform = YUAN_PLUS_DMI_ATTO3;

  //   ret = BUILD_SAFETY_CFG(byd_yuanplus_atto3_rx_checks, BYD_YUANPLUS_ATTO3_TX_MSGS);
  // } else {
    // byd_platform = HAN_TANG_DMEV;
    ret = BUILD_SAFETY_CFG(byd_han_dmev_rx_checks, BYD_HAN_DMEV_TX_MSGS);
  // }

  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
