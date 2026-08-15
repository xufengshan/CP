#!/usr/bin/env python3
import argparse
import time

from typing import Any
from multiprocessing import Queue

from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

def create_bridge(dual_camera, high_quality):
  queue: Any = Queue()

  simulator_bridge = MetaDriveBridge(dual_camera, high_quality)
  simulator_process = simulator_bridge.run(queue)

  return queue, simulator_process, simulator_bridge

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between the simulator and openpilot.')
  parser.add_argument('--joystick', action='store_true')
  parser.add_argument('--high_quality', action='store_true')
  parser.add_argument('--dual_camera', action='store_true')

  return parser.parse_args(add_args)

if __name__ == "__main__":
  args = parse_args()

  queue, simulator_process, simulator_bridge = create_bridge(args.dual_camera, args.high_quality)

  # Try to start keyboard/joystick input, but do NOT let a failure (e.g. no TTY
  # when launched via nohup/ssh) shut down the bridge. If input is unavailable,
  # just keep the simulator process alive so simulated_car keeps publishing CAN.
  try:
    if args.joystick:
      from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread
      wheel_poll_thread(queue)
    else:
      from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
      keyboard_poll_thread(queue)
  except Exception as e:
    print(f"[sim] input unavailable ({e}), running headless")
    try:
      while simulator_process.is_alive():
        time.sleep(0.5)
    except KeyboardInterrupt:
      pass

  simulator_bridge.shutdown()
  simulator_process.join()
