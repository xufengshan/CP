import traceback
import openpilot.cereal.messaging as messaging

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.byd.values import BydSafetyFlags
from openpilot.common.params import Params
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.tools.sim.lib.common import SimulatorState


class SimulatedCar:
  """Simulates a BYD TANG DM (panda state + can messages) to OpenPilot"""
  packer = CANPacker("byd_han_dmev_2020")

  def __init__(self):
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'carParams', 'selfdriveState'])
    self.cp = self.get_car_can_parser()
    self.idx = 0
    self.params = Params()
    self.obd_multiplexing = False
    self._cnt_pt = 0
    self._cnt_cam = 0

  @staticmethod
  def get_car_can_parser():
    dbc_f = 'byd_han_dmev_2020'
    checks = []
    return CANParser(dbc_f, checks, 0)

  def send_can_messages(self, simulator_state: SimulatorState):
    if not simulator_state.valid:
      return

    msg = []
    speed = simulator_state.speed * 3.6  # convert m/s to kph

    # *** powertrain bus (CanBus.ESC = 0) ***
    # CARSPEED @ 50Hz
    msg.append(self.packer.make_can_msg("CARSPEED", 0, {
      "CarDisplaySpeed": speed,
    }))

    # DRIVE_STATE @ 50Hz - Gear 4 = D
    msg.append(self.packer.make_can_msg("DRIVE_STATE", 0, {
      "Gear": 4,  # D
      "Counter": self._cnt_pt & 0xF,
      "BrakePressed": 1 if simulator_state.user_brake > 0 else 0,
    }))

    # EPS @ 100Hz
    msg.append(self.packer.make_can_msg("EPS", 0, {
      "SteeringAngle": simulator_state.steering_angle,
      "Counter": self.idx & 0xFF,
    }))

    # ACC_EPS_STATE @ 50Hz
    msg.append(self.packer.make_can_msg("ACC_EPS_STATE", 0, {
      "CruiseActivated": 1 if simulator_state.is_engaged else 0,
      "MainTorque": 0,
      "SteerDriverTorque": int(simulator_state.user_torque) if abs(simulator_state.user_torque) < 2048 else 0,
      "Counter": self._cnt_pt & 0xF,
    }))

    # PCM_BUTTONS @ 20Hz
    msg.append(self.packer.make_can_msg("PCM_BUTTONS", 0, {
      "BTN_TOGGLE_ACC_OnOff": 1,
      "BTN_AccCancel": 0,
      "BTN_AccUpDown_Cmd": 0,
      "Counter": self._cnt_pt & 0xF,
    }))

    # STALKS @ 1Hz
    msg.append(self.packer.make_can_msg("STALKS", 0, {
      "HeadLight": 1,
      "LeftIndicator": 1 if simulator_state.left_blinker else 0,
      "RightIndicator": 1 if simulator_state.right_blinker else 0,
    }))

    # YAW_RATE @ 50Hz
    msg.append(self.packer.make_can_msg("YAW_RATE", 0, {
      "YawRate": 0.0,
      "YawRateOffset": 0.0,
      "Counter": self._cnt_pt & 0xF,
    }))

    # PEDAL @ 50Hz
    msg.append(self.packer.make_can_msg("PEDAL", 0, {
      "AcceleratorPedal": int(simulator_state.user_gas * 100),
      "BrakePedal": int(simulator_state.user_brake * 100),
    }))

    # EPB @ 1Hz
    msg.append(self.packer.make_can_msg("EPB", 0, {
      "EPB_ActiveFlag": 0,
    }))

    # BELT @ 20Hz - fastened = 2
    msg.append(self.packer.make_can_msg("BELT", 0, {
      "SeatBeat": 2,
    }))

    # BCM @ 1Hz
    msg.append(self.packer.make_can_msg("BCM", 0, {
      "FrontLeftDoor": 0,
      "FrontRightDoor": 0,
      "RearLeftDoor": 0,
      "RearRightDoor": 0,
      "BootDoor": 0,
    }))

    # DATETIME @ 2Hz
    msg.append(self.packer.make_can_msg("DATETIME", 0, {
      "YY": 26, "MM": 8, "DD": 15, "hh": 16, "mm": 0, "ss": 0,
    }))

    # *** cam bus (CanBus.MPC = 2) ***
    # ACC_HUD_ADAS @ 50Hz
    msg.append(self.packer.make_can_msg("ACC_HUD_ADAS", 2, {
      "SetSpeed": 40.0,
      "HasLead": 0,
      "SetDistance": 2,
      "AccState": 1 if simulator_state.is_engaged else 0,
      "AccOn1": 1,
      "Counter": self._cnt_cam & 0xF,
    }))

    # ACC_CMD @ 50Hz
    msg.append(self.packer.make_can_msg("ACC_CMD", 2, {
      "AccelCmd": 0.0,
      "Counter": self._cnt_cam & 0xF,
    }))

    # ACC_MPC_STATE @ 50Hz
    msg.append(self.packer.make_can_msg("ACC_MPC_STATE", 2, {
      "LKAS_Config": 1,
      "LKAS_Output": 0,
      "LKAS_ReqPrepare": 0,
      "LKAS_Active": 1,
      "Counter": self._cnt_cam & 0xF,
    }))

    self._cnt_pt += 1
    self._cnt_cam += 1

    # bus1 雷达空闲帧 (Continental ARS4xx, CAN_BUS=1, 0x380-0x3FF, radar_interface.py)
    # dat[3]=0xFF = 无前车; 之前 SP 版本 2026-08-12 同款修复
    msg.append((0x380, bytes([0x07,0x00,0x00,0xFF,0x00,0x00,0x00,0x00]), 1))

    self.pm.send('can', can_list_to_can_capnp(msg))

  def send_panda_state(self, simulator_state):
    self.sm.update(0)

    if self.params.get_bool("ObdMultiplexingEnabled") != self.obd_multiplexing:
      self.obd_multiplexing = not self.obd_multiplexing
      self.params.put_bool("ObdMultiplexingChanged", True, block=True)

    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': simulator_state.ignition,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'byd',
      'alternativeExperience': self.sm["carParams"].alternativeExperience,
      'safetyParam': BydSafetyFlags.HAN_TANG_DMEV.value,
    }
    self.pm.send('pandaStates', dat)

  def update(self, simulator_state: SimulatorState):
    try:
      self.send_can_messages(simulator_state)

      if self.idx % 50 == 0:  # only send panda states at 2hz
        self.send_panda_state(simulator_state)

      self.idx += 1
    except Exception:
      traceback.print_exc()
