"""Example of using the MitPositionController to move the arm to a fixed pose.

To run this example:
python3 scripts/simple_move.py
"""

import time

from piper_control import (
    piper_connect,
    piper_control,
    piper_init,
    piper_interface,
)

import numpy as np

def main():
  print(
      "This script will move the 2nd-to-last joint of the robot arm a small "
      "amount, using position control."
  )
  input("WARNING: the robot will move. Press Enter to continue...")

  ports = piper_connect.find_ports()
  print(f"Piper ports: {ports}")

  piper_connect.activate(ports)
  ports = piper_connect.active_ports()

  if not ports:
    raise ValueError(
        "No ports found. Make sure the Piper is connected and turned on. "
        "If you are having issues connecting to the piper, check our "
        "troubleshooting guide @ "
        "https://github.com/Reimagine-Robotics/piper_control/blob/main/README.md"
    )

  robot = piper_interface.PiperInterface(can_port=ports[0])
  robot.set_installation_pos(piper_interface.ArmInstallationPos.UPRIGHT)

  print("resetting arm")
  piper_init.reset_arm(
      robot,
      arm_controller=piper_interface.ArmController.MIT,
      move_mode=piper_interface.MoveMode.MIT,
  )

  # print("resetting gripper")
  # piper_init.reset_gripper(robot)

  robot.show_status()
  # robot.get_joint_positions()
  # while 1:
  #   print(robot.get_joint_positions())
  # First open and close the gripper
  # with piper_control.GripperController(robot) as controller:
  #   controller.command_open()
  #   time.sleep(2.0)
  #   controller.command_close()
  #   time.sleep(2.0)

  # Move the arm joints using Mit mode controller.
  cmd_joint_pos = np.array(robot.get_joint_positions(), dtype=np.float32)
  # target_joint_pos = np.array([0.5, 0.7, -0.4, 0.2, 0.3, 0.5], dtype=np.float32)
  target_joint_pos = np.array([0.0, 0.0, -0.0, 0.0, 0.0, 0.0], dtype=np.float32)
  with piper_control.MitJointPositionController(
      robot,
      kp_gains=5.0,
      kd_gains=0.8,
      rest_position=piper_control.ArmOrientations.upright.rest_position,
  ) as controller:
    print("moving to position ...")
    while 1:
      success = controller.move_to_position(
          # [0.5, 0.7, -0.4, 0.2, 0.3, 0.5],
          cmd_joint_pos.tolist(),
          threshold=0.05,
          timeout=0.01,
      )
      cmd_joint_pos += np.clip(target_joint_pos-cmd_joint_pos, a_min=-0.02, a_max=0.02)
      time.sleep(0.01)
    print(f"reached target: {success}")

  print("finished, disabling arm.")
  print("WARNING: the arm will power off and drop.")

  piper_init.disable_arm(robot)
  # piper_init.disable_gripper(robot)
  time.sleep(0.5)
  print("done disabling. exiting.")


if __name__ == "__main__":
  main()
