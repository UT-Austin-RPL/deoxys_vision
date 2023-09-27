import time

import cv2
import gprs.utils.transform_utils as T
import init_path
import numpy as np

from deoxys_vision.k4a.k4a_interface import K4aInterface
from deoxys_vision.utils.apriltag_detector import AprilTagDetector

k4a_interface = K4aInterface()

k4a_interface.start()

while True:
    capture = k4a_interface.get_last_obs()
    if capture is None:
        continue

    image = capture["color"].astype(np.uint8)
    intrinsics = k4a_interface.get_color_intrinsics()

    intrinsics = {
        "cx": 638.838195,
        "cy": 367.01812744,
        "fx": 612.7537841796875,
        "fy": 612.6171264648438,
    }

    # Get AprilTag Detection

    april_detector = AprilTagDetector()
    april_detector.detect(image, intrinsics=intrinsics, tag_size=0.06)

    image = april_detector.vis_tag(image)

    cv2.imshow("", image)
    cv2.waitKey(1)

    if len(april_detector.results) == 1:
        print(april_detector.results[0].tag_id)
        print("Translation: ", april_detector.results[0].pose_t)
        # print("Rotation: ", april_detector.results[0].pose_R)
        print("Rotation: ", T.mat2quat(april_detector.results[0].pose_R.transpose()))
        print("=========================================")
        time.sleep(0.05)


# for detection in april_detector.results:
#     print(detection)
#     print("=================================")
