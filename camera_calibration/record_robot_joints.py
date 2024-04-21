"""This is a script to record joints by moving robot around, and save it to a list"""
import os
import time
import numpy as np
import simplejson as json

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse

config_folder = os.path.join(os.path.expanduser("~/"), ".deoxys_vision/calibration_configuration")
os.makedirs(os.path.join(os.path.expanduser("~/"), config_folder), exist_ok=True)


def main():

    device = SpaceMouse(vendor_id=9583, product_id=50734)
    device.start_control()

    # print(config_root)
    robot_interface = FrankaInterface(config_root + "/charmander.yml", use_visualizer=False)
    controller_cfg = YamlConfig(config_root + "/compliant-joint-impedance-controller.yml").as_easydict()
    controller_type = "JOINT_IMPEDANCE"

    # # Make it low impedance so that we can easily move the arm around
    # controller_cfg["Kp"]["translation"] = 50
    # controller_cfg["Kp"]["rotation"] = 50

    joints = []

    recorded_joint = False
    time.sleep(1.)    
    while True:
        spacemouse_action, grasp = input2action(
            device=device,
            controller_type="OSC_POSE",
        )

        if spacemouse_action is None:
            break

        if len(robot_interface._state_buffer) > 0:
            if spacemouse_action[-1] > 0 and not recorded_joint:
                joints.append(robot_interface._state_buffer[-1].q)
                print(len(robot_interface._state_buffer[-1].q))
                recorded_joint = True
                for _ in range(5):
                    spacemouse_action, grasp = input2action(
                        device=device,
                        controller_type=controller_type,
                    )
            elif spacemouse_action[-1] < 0:
                recorded_joint = False
        else:
            continue
        action = list(robot_interface._state_buffer[-1].q) + [-1]
        robot_interface.control(
            controller_type=controller_type, action=action, controller_cfg=controller_cfg
        )

    save_joints = []
    for joint in joints:
        if np.linalg.norm(joint) < 1.0:
            continue
        # print(joint)
        save_joints.append(np.array(joint).tolist())

    while True:
        try:
            save = int(input("save or not? (1 - Yes, 0 - No)"))
        except ValueError:
            print("Please input 1 or 0!")
            continue
        break

    if save:
        file_name = input("Filename to save the joints: ")
        joint_info_json_filename = f"{config_folder}/{file_name}.json"

        with open(joint_info_json_filename, "w") as f:
            json.dump({"joints": save_joints}, f, indent=4)
        print(f"Saving to {file_name}.json")

    robot_interface.close()


if __name__ == "__main__":
    main()
