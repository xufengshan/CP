#pragma once

#include "safety_declarations.h"

// ============================================================================
// BYD safety mode
// ============================================================================
// Ported/adapted from the reference (carrot cp11) safety_byd.h, learning only
// the well-understood torque-control branch used by BYD Tang DM / Han / Song
// family. Aggressive/experimental additions (ANGLE_MODE, QINPLUS, external GM
// radar keepalive, SEAL frames) are intentionally NOT ported to keep behavior
// identical to the verified in-repo BYD torque controller and avoid regressions.
//
// CAN address conventions (match byd_han_dmev_2020.dbc):
//   PEDAL            0x342  (834)
//   CARSPEED         0x121  (289)
//   EPS              0x11F  (287)
//   ACC_EPS_STATE    0x318  (792)
//   ACC_HUD_ADAS     0x32D  (813, bus MPC)
//   ACC_MPC_STATE    0x316  (790, bus MPC)
//   ACC_CMD          0x32E  (814, bus ESC)
//   PCM_BUTTONS      0x3B0  (944, bus MPC)
//   DRIVE_STATE      0x242  (578)
// Bus: ESC=0, MRR=1, MPC=2
// ============================================================================

#define BYD_CANADDR_ACC_MPC_STATE  0x316
#define BYD_CANADDR_ACC_EPS_STATE  0x318
#define BYD_CANADDR_ACC_HUD_ADAS   0x32D
#define BYD_CANADDR_ACC_CMD        0x32E
#define BYD_CANADDR_PCM_BUTTONS    0x3B0
#define BYD_CANADDR_DRIVE_STATE    0x242
#define BYD_CANADDR_PEDAL          0x342
#define BYD_CANADDR_EPS            0x11F
#define BYD_CANADDR_CARSPEED       0x121

#define BYD_CANBUS_ESC  0               // ESC bus
#define BYD_CANBUS_MRR  1               // radar bus
#define BYD_CANBUS_MPC  2               // MPC bus

// ---------------------------------------------------------------------------
// RX hook
// ---------------------------------------------------------------------------
static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == BYD_CANBUS_ESC) {
    if (addr == BYD_CANADDR_PEDAL) {
      gas_pressed = (GET_BYTE(to_push, 0) != 0U);
      brake_pressed = (GET_BYTE(to_push, 1) != 0U);
    } else if (addr == BYD_CANADDR_CARSPEED) {
      int speed_raw = ((GET_BYTE(to_push, 1) & 0x0FU) << 8) | GET_BYTE(to_push, 0);
      vehicle_moving = (speed_raw != 0);
      UPDATE_VEHICLE_SPEED(speed_raw * 0.01997467f); //unit: m/s
    }
  } else if (bus == BYD_CANBUS_MPC) {
    if (addr == BYD_CANADDR_ACC_HUD_ADAS) {
      // 3 = acc active, 5 = user force accel
      unsigned int accstate = ((GET_BYTE(to_push, 2) >> 3) & 0x07U);
      pcm_cruise_check((accstate == 0x3U) || (accstate == 0x5U));
    }
  }

  generic_rx_checks((addr == BYD_CANADDR_ACC_MPC_STATE) && (bus == BYD_CANBUS_ESC));
}

// ---------------------------------------------------------------------------
// TX hook
// ---------------------------------------------------------------------------
static bool byd_tx_hook(const CANPacket_t *to_send) {
  // Torque steering limits (match BYD torque controller; safety ceiling safely
  // above controller max (STEER_MAX=280), per reference values).
  const TorqueSteeringLimits BYD_TORQUE_STEERING_LIMITS = {
    .max_steer = 300,                     // max steering value
    .max_rate_up = 18,                    // max rate up
    .max_rate_down = 18,                  // max rate down
    .max_rt_delta = 250,                  // max real-time delta
    .max_rt_interval = 250000,            // 250ms
    .max_torque_error = 80,               // motor torque limits
    .type = TorqueMotorLimited,           // limit type
  };

  bool tx = true;

  if (GET_BUS(to_send) == BYD_CANBUS_ESC) {
    int addr = GET_ADDR(to_send);

    if (addr == BYD_CANADDR_ACC_MPC_STATE) {
      int desired_torque = ((GET_BYTE(to_send, 3) & 0x07U) << 8U) | GET_BYTE(to_send, 2);
      desired_torque = to_signed(desired_torque, 11);
      bool steer_req = GET_BIT(to_send, 28U); //LKAS_Active

      if (steer_torque_cmd_checks(desired_torque, steer_req, BYD_TORQUE_STEERING_LIMITS)) {
        tx = true;
      }
    }
  }

  return tx;
}

// ---------------------------------------------------------------------------
// Forward hook
// ---------------------------------------------------------------------------
static int byd_fwd_hook(CANPacket_t *to_send) {
  const int bus = GET_BUS(to_send);
  const int addr = GET_ADDR(to_send);
  int bus_fwd = -1;

  if (bus == BYD_CANBUS_ESC) {
    bool block_esc_msg = (addr == BYD_CANADDR_ACC_EPS_STATE)
                      || (addr == BYD_CANADDR_PCM_BUTTONS);

    if (!block_esc_msg) {
      bus_fwd = BYD_CANBUS_MPC;
    }
  } else if (bus == BYD_CANBUS_MPC) {
    bool block_mpc_msg = (addr == BYD_CANADDR_ACC_HUD_ADAS)
                      || (addr == BYD_CANADDR_ACC_MPC_STATE)
                      || (addr == BYD_CANADDR_ACC_CMD);

    if (!block_mpc_msg) {
      bus_fwd = BYD_CANBUS_ESC;
    }
  }

  return bus_fwd;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
static safety_config byd_init(uint16_t param) {
  UNUSED(param);

  safety_config ret;

  static RxCheck byd_rx_checks[] = {
    {.msg = {{BYD_CANADDR_PEDAL,         BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_CARSPEED,      BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_EPS_STATE, BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_HUD_ADAS,  BYD_CANBUS_MPC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_MPC_STATE, BYD_CANBUS_MPC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  };

  static const CanMsg BYD_TX_MSGS[] = {
    {BYD_CANADDR_ACC_CMD,                 BYD_CANBUS_ESC, 8},
    {BYD_CANADDR_ACC_HUD_ADAS,            BYD_CANBUS_ESC, 8},
    {BYD_CANADDR_ACC_MPC_STATE,           BYD_CANBUS_ESC, 8},
    {BYD_CANADDR_ACC_EPS_STATE,           BYD_CANBUS_MPC, 8},
    {BYD_CANADDR_PCM_BUTTONS,             BYD_CANBUS_MPC, 8},
  };

  ret = BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);

  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
