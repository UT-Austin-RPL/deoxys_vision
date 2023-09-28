import argparse
import json
import os
import struct
import time

import cv2
import init_path
import numpy as np
import redis
from easydict import EasyDict

from deoxys_vision.networking.camera_redis_interface import CameraRedisPubInterface
from deoxys_vision.camera.k4a_interface import K4aInterface
from deoxys_vision.camera.rs_interface import RSInterface
from deoxys_vision.img_utils import preprocess_color, preprocess_depth


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-type", type=str, choices=["k4a", "rs"])
    parser.add_argument("--camera-id", type=int, default=0)

    parser.add_argument("--eval", action="store_true")

    parser.add_argument("--use-rgb", action="store_true")
    parser.add_argument("--use-depth", action="store_true")
    parser.add_argument("--use-rec", action="store_true")

    parser.add_argument("--rgb-convention", default="rgb", choices=["bgr", "rgb"])

    # parser.add_argument("--no-color", action="store_true")

    # parser.add_argument("--no-depth", action="store_true")

    parser.add_argument("--visualization", action="store_true")

    parser.add_argument("--depth-visualization", action="store_true")

    args = parser.parse_args()

    # print information about the cameras to run

    camera_config = EasyDict(
        camera_type=args.camera_type,
        camera_id=args.camera_id,
        use_rgb=args.use_rgb,
        use_depth=args.use_depth,
        use_rec=args.use_rec,
        rgb_convention=args.rgb_convention,
    )
    print("This node runs with the camera {camera_cofnig.camera_type} with id {camera_config.camera_id}")
    print("The node will publish the following data:")
    if args.use_rgb:
        print("- Color image")
    if args.use_depth:
        print("- Depth image")
    if args.use_rec:
        print("Note that Images are rectified with undistortion")

    camera_id = args.camera_id

    host = "172.16.0.1"
    port = 6379
    camera2redis_pub_interface = CameraRedisPubInterface(
        redis_host=host, redis_port=port, camera_id=camera_id,
        custom_camera_name=f"{args.camera_type}_camera_{camera_id}"
    )
    # Check redis if the camera id is occupied or not.
    camera_interface = None

    node_config = EasyDict(use_color=True, use_depth=True)
    if not args.use_rgb:
        node_config.use_color = False

    if not args.use_depth:
        node_config.use_depth = False

    if args.camera_type == "k4a":
        camera_interface = K4aInterface()
    elif args.camera_type == "rs":
        import pyrealsense2 as rs

        color_cfg = EasyDict(
            enabled=node_config.use_color, img_w=640, img_h=480, img_format=rs.format.bgr8, fps=30
        )

        depth_cfg = EasyDict(
            enabled=node_config.use_depth, img_w=640, img_h=480, img_format=rs.format.z16, fps=30
        )

        pc_cfg = EasyDict(enabled=False)
        camera_interface = RSInterface(
            device_id=camera_id, color_cfg=color_cfg, depth_cfg=depth_cfg, pc_cfg=pc_cfg
        )

    camera_interface.start()
    print("Starting")
    t = time.time()
    save_dir = f"/tmp/{args.camera_type}_{camera2redis_pub_interface.camera_name}_{t}"
    file_ext = "jpg"
    camera_num = 0
    os.makedirs(save_dir)

    COUNT_THRESH = 5
    counter = COUNT_THRESH

    img_counter = 0
    freq = 50.0
    while True:
        start_time = time.time_ns()

        capture = camera_interface.get_last_obs()

        if capture is None:
            continue

        # t = time.time_ns()
        if capture is None:
            continue

        save_img = camera2redis_pub_interface.get_save_img_info()

        if save_img:
            counter = 0
        else:
            counter += 1
        img_info = {}
        imgs = {}
        img_info["time"] = t
        img_info["camera_type"] = args.camera_type

        img_info["intrinsics"] = {}
        if node_config.use_color:
            color_img = preprocess_color(capture["color"], flip_channel=camera_config.rgb_convention == "rgb")
            color_img_name = f"{save_dir}/color_{img_counter:09d}.{file_ext}"
            img_info["color_img_name"] = color_img_name
            img_info["intrinsics"]["color"] = camera_interface.get_color_intrinsics(mode="dict")
            # img_info["distortion"]["color"] = camera_interface.get_color_distortion()
            intrinsics_matrix = camera_interface.get_color_intrinsics(mode="matrix")
            color_distortion = camera_interface.get_color_distortion()

            if camera_config.use_rec:
                imgs["color"] = cv2.undistort(color_img, intrinsics_matrix, color_distortion, None)
            else:
                imgs["color"] = color_img

        if node_config.use_depth:
            depth_img = preprocess_depth(capture["depth"])
            depth_img_name = f"{save_dir}/depth_{img_counter:09d}.{file_ext}"
            img_info["depth_img_name"] = depth_img_name
            img_info["intrinsics"]["depth"] = camera_interface.get_depth_intrinsics(mode="dict")
            depth_distortion = camera_interface.get_depth_distortion()
            intrinsics_depth_matrix = camera_interface.get_depth_intrinsics(mode="matrix")            
            imgs["depth"] = depth_img

        camera2redis_pub_interface.set_img_info(img_info)
        camera2redis_pub_interface.set_img_buffer(imgs=imgs)

        # if not args.eval:
        #     if counter < COUNT_THRESH:
        #         # Save img to tmp file
        #         if node_config.use_color:
        #             cv2.imwrite(color_img_name, imgs["color"])
        #         if node_config.use_depth:
        #             cv2.imwrite(depth_img_name, imgs["depth"])
        #         img_counter += 1
        # if not args.eval:
        #     # Save img to tmp file
        #     if node_config.use_color:
        #         cv2.imwrite(color_img_name, imgs["color"])
        #     if node_config.use_depth:
        #         cv2.imwrite(depth_img_name, imgs["depth"])

        # r.set(f"camera_{camera_num}::last_depth_img", depth_img.tobytes())
        if args.visualization:
            cv2.imshow("", imgs["color"])
            if args.depth_visualization:
                cv2.imshow("depth", imgs["depth"] * 0.001)
            cv2.waitKey(10)
        end_time = time.time_ns()

        time_interval = (end_time - start_time) / (10 ** 9)
        if time_interval < 1.0 / freq:
            time.sleep(1.0 / freq - time_interval)

        if camera2redis_pub_interface.finished:
            break


if __name__ == "__main__":
    main()
