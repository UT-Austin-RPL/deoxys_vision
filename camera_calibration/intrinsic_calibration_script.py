import argparse
import json
import os
import pprint
import time

import cv2
import gprs.utils.transform_utils as T
import init_path
import numpy as np
from gprs import config_root
from gprs.camera_redis_interface import CameraRedisSubInterface
from gprs.franka_interface import FrankaInterface
from gprs.franka_interface.visualizer import PybulletVisualizer
from gprs.utils import load_yaml_config
from gprs.utils.input_utils import input2action
from gprs.utils.io_devices import SpaceMouse


def read_chessboards(images, board):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    for img_idx, img in enumerate(images):
        print("=> Processing image {img_idx}")
        # frame = cv2.imread(im)
        frame = img
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)
        if len(corners) > 0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(
                    gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria
                )
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if (
                res2[1] is not None
                and res2[2] is not None
                and len(res2[1]) > 3
                and decimator % 1 == 0
            ):
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator += 1

    imsize = gray.shape
    return allCorners, allIds, imsize

def calibrate_camera(allCorners, allIds, imsize, board):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    cameraMatrixInit = np.array(
        [[1000.0, 0.0, imsize[0] / 2.0], [0.0, 1000.0, imsize[1] / 2.0], [0.0, 0.0, 1.0]]
    )

    distCoeffsInit = np.zeros((5, 1))
    flags = (
        cv2.CALIB_USE_INTRINSIC_GUESS
        + cv2.CALIB_RATIONAL_MODEL
        + cv2.CALIB_FIX_ASPECT_RATIO
    )
    # flags = (cv2.CALIB_RATIONAL_MODEL)
    (
        ret,
        camera_matrix,
        distortion_coefficients0,
        rotation_vectors,
        translation_vectors,
        stdDeviationsIntrinsics,
        stdDeviationsExtrinsics,
        perViewErrors,
    ) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9),
    )

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

def main():

    device = SpaceMouse(vendor_id=9583, product_id=50734)
    device.start_control()
    
    camera_id = 0
    cr_interface = CameraRedisSubInterface(camera_id=camera_id, use_depth=False)
    cr_interface.start()
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard_create(
        7,
        5,
        squareLength=1, # 0.03785, # 0.2863,
        markerLength=0.8, # 0.0303,# 0.2308,
        # 1,
        # .8,
        dictionary=aruco_dict,
    )

    recorded_image = False
    time.sleep(0.2)

    images = []
    while True:
        spacemouse_action, grasp = input2action(
            device=device,
            control_type="OSC_POSE",
        )

        if spacemouse_action is None:
            break

        if spacemouse_action[-1] > 0 and not recorded_image:
            time.sleep(0.2)
            imgs = cr_interface.get_img()
            color_img = imgs["color"]
            images.append(color_img)
            recorded_image = True
            for _ in range(5):
                spacemouse_action, grasp = input2action(
                    device=device,
                    control_type="OSC_POSE",
                )
        elif spacemouse_action[-1] < 0:
            recorded_image = False
    
    # img_info = cr_interface.get_img_info()
    # print(img_info["intrinsics"]["color"])

    allCorners, allIds, imsize = read_chessboards(images, board)
    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners, allIds, imsize, board)

    print("ret: ", ret)
    print("dist: ", dist)
    

    for idx in range(len(images)):
    # for color_img, allCorner, allId in zip(images, allCorners, allIds):
        img = cv2.aruco.drawDetectedCornersCharuco(images[idx], allCorners[idx], allIds[idx])
        cv2.imshow("", img)
        cv2.waitKey(0)


if __name__ == "__main__":
    main()
