import argparse
import json
import os
import pprint
import time

import cv2
import deoxys.utils.transform_utils as T
import init_path
import numpy as np
from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.franka_interface.visualizer import PybulletVisualizer
from deoxys.utils import load_yaml_config
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse

from deoxys_vision.utils.camera_utils import assert_camera_ref_convention, get_camera_info
from deoxys_vision.networking.camera_redis_interface import CameraRedisSubInterface
from deoxys_vision.experimental.calibration import EyeInHandCalibration, EyeToHandCalibration

# folder_path = os.path.join(os.path.dirname(__file__))


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--config-folder",
        type=str,
        default=os.path.expanduser("~/.deoxys_vision/calibration_configuration"),
    )

    parser.add_argument(
        "--result-folder",
        type=str,
        default=os.path.expanduser("~/.deoxys_vision/calibration"),
    )
    
    parser.add_argument("--config-filename", type=str, default="joints_info.json")

    parser.add_argument("--camera-ref", type=str)

    parser.add_argument("--use-saved-images", action="store_true")

    parser.add_argument("--debug", action="store_true")

    parser.add_argument("-p", "--post-fix", type=str, default="")

    parser.add_argument("--calibration-type", type=str, choices=["eye-to-hand", "eye-in-hand"])
    
    return parser.parse_args()


def main():

    args = parse_args()
    assert_camera_ref_convention(args.camera_ref)
    camera_info = get_camera_info(args.camera_ref)


    # load a list of joints to move
    # joints_json_file_name = f"{args.config_folder}/{args.config_filename}"
    joints_json_file_name = args.config_filename

    joint_info = json.load(open(joints_json_file_name, "r"))
    joint_list = joint_info["joints"]

    joint_list = joint_list
    # Iniitalize camera interface

    use_saved_images = args.use_saved_images

    new_joint_list = []

    calibration_img_folder = "calibration_imgs"

    # Image Collection
    if not use_saved_images:
        os.makedirs(calibration_img_folder, exist_ok=True)
        camera_id = camera_info.camera_id
        cr_interface = CameraRedisSubInterface(camera_info=camera_info, use_depth=True)
        cr_interface.start()

        # Load robot controller configs
        controller_cfg = load_yaml_config(config_root + "/joint-position-controller.yml")
        robot_interface = FrankaInterface(config_root + "/charmander.yml", use_visualizer=False)
        controller_type = "JOINT_POSITION"

        intrinsics = cr_interface.get_img_info()["intrinsics"]
        print(intrinsics)

        for (idx, robot_joints) in enumerate(joint_list):
            action = robot_joints + [-1]

            while True:
                if len(robot_interface._state_buffer) > 0:
                    if (
                        np.max(
                            np.abs(
                                np.array(robot_interface._state_buffer[-1].q)
                                - np.array(robot_joints)
                            )
                        )
                        < 5e-3
                    ):
                        break
                robot_interface.control(
                    controller_type=controller_type, action=action, controller_cfg=controller_cfg
                )

            # save image
            time.sleep(0.8)
            while not np.linalg.norm(robot_interface._state_buffer[-1].q) > 0:
                time.sleep(0.01)
            new_joint_list.append(robot_interface._state_buffer[-1].q)
            imgs = cr_interface.get_img()
            img_info = cr_interface.get_img_info()

            color_img = imgs["color"]
            depth_img = imgs["depth"]
            cv2.imshow("", color_img[..., ::-1])
            cv2.imwrite(f"{calibration_img_folder}/{idx}_color.png", color_img)
            cv2.imwrite(f"{calibration_img_folder}/{idx}_depth.png", depth_img)
            cv2.waitKey(1)

            time.sleep(0.3)

        with open(
            os.path.join(args.config_folder, f"{camera_info.camera_name}.json"),
            "w",
        ) as f:
            json.dump(intrinsics, f)
        # joint_list = new_joint_list
        robot_interface.close()

    # Start Calibration
    if args.calibration_type == "eye-in-hand":
        calibration_class_name = "EyeInHandCalibration"
        # hard-coded value, adjust based on your need
        tag_size = 0.080
    elif args.calibration_type == "eye-to-hand":
        calibration_class_name = "EyeToHandCalibration"
        # hard-coded value, adjust based on your need
        tag_size = 0.05931

    with open(
        os.path.join(args.config_folder, f"{camera_info.camera_name}.json"), "r"
    ) as f:
        intrinsics = json.load(f)

    print(intrinsics)
    handeye_calibration = eval(calibration_class_name)()
    handeye_calibration.reset()
    handeye_calibration.update_detector_configs(intrinsics=intrinsics["color"],
                                                tag_size=tag_size)
    
    imgs = []

    for (idx, robot_joints) in enumerate(new_joint_list):
        img = cv2.imread(f"{calibration_img_folder}/{idx}_color.png")
        handeye_calibration.step(img, robot_joints, verbose=False)

    results = {}
    for calibration_method in ["tsai", 'horaud']:
        results[calibration_method] = {"pos": [], "rot": []}
        pos, rot = handeye_calibration.calibrate(calibration_method=calibration_method,
                                      verbose=True)
        results[calibration_method]["pos"] = pos
        results[calibration_method]["rot"] = rot
    # Save calibration result
    for calibration_method in results.keys():
        output_json_file_name =  os.path.join(
                args.result_folder,
                f"{camera_info.camera_name}_{calibration_method}_extrinsics.json",
            )
        with open(
           output_json_file_name,
            "w",
        ) as f:
            extrinsics = {"translation": results[calibration_method]["pos"].tolist(),
                          "rotation": results[calibration_method]["rot"].tolist()}
            json.dump(extrinsics, f)

        print(f"calibration result saved at: {output_json_file_name}")
    
    # for idx in range(len(joint_list)):
    #     rpl_transform_manager.add_transform(f"cam_view_{idx}", f"ee_{idx}", R, t)

    # print("==============================")
    # for idx in range(len(joint_list)):
    #     pp.pprint(f"View {idx}:")
    #     target2base = rpl_transform_manager.get_transform(f"target_{idx}", "base")
    #     if target2base is not None:
    #         pp.pprint(np.round(target2base, 3))

if __name__ == "__main__":
    main()
