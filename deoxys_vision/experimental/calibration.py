import os
import json
import cv2

import numpy as np

from deoxys_vision import ROOT_PATH
from deoxys_vision.utils.markers.apriltag_detector import AprilTagDetector
from deoxys_vision.utils.transformation.transform_manager import TransformManager
from deoxys_vision.utils.robot_urdf_models.urdf_models import PandaURDFModel
import deoxys_vision.utils.transformation.transform_utils as T

class HandEyeCalibrationBase():
    def __init__(self,
                 asset_folder=os.path.join(ROOT_PATH, "utils/robot_urdf_models"),
                 assistive_marker_type="apriltag",
                 robot="Panda"):
        assert(assistive_marker_type in ["apriltag"])
        assert(robot in ["Panda"])

        self._assistive_marker_type = assistive_marker_type
        if self._assistive_marker_type == "apriltag":
            self._marker_detector = AprilTagDetector()
            self._marker_configuration = {"intrinsics": None, "tag_size": None}
        self._transform_manager = TransformManager()

        self._robot_model = PandaURDFModel(asset_folder=asset_folder)

        self.rot_target2cam_list = []
        self.pos_target2cam_list = []

        self.counter = 0

    def reset(self):
        self.rot_target2cam_list = []
        self.pos_target2cam_list = []
        self.counter = 0

    def update_detector_configs(self, **kwargs):
        self._marker_configuration.update(kwargs)

    def detect_marker(self, rgb_img):
        """
        Args:
           rgb_img (np.array): HxWx3 RGB images for detecting markers
        Return:
           detected pose of the marker, in a tuple (R, t)
        """
        detect_results = self._marker_detector.detect(rgb_img, **self._marker_configuration)
        if len(detect_results) != 1:
            print(f"wrong detection, skipping detection of the current image")
            return None
        else:
            rot = np.array(detect_results[0].pose_R)
            pos = np.array(detect_results[0].pose_t)
            return (pos, rot)

    def get_gripper_pose_from_model(self, joint_configuration):
        pose = self._robot_model.get_gripper_pose(joint_configuration)[:2]
        pos = np.array(pose[0])
        rot = np.array(T.quat2mat(pose[1]))
        return (pos, rot)

    def get_marker_pose_from_model(self, joint_configuration):
        """
        The robot model has the model of marker, which can be used to compute the pose of the marker based on the robot model.
        """
        pose = self._robot_model.get_marker_pose(joint_configuration)[:2]
        pos = np.array(pose[0])
        rot = np.array(T.quat2mat(pose[1]))
        return (pos, rot)
    
    def add_transform(self, target_frame_name, source_frame_name, R, pos):
        """
        Store the transformation of `target_frame` relative to `source_frame`
        """
        self._transform_manager.add_transform(target_frame_name, source_frame_name, R, pos)

    def get_transform(self, target_frame_name, source_frame_name):
        """
        Store the transformation of `target_frame` relative to `source_frame`
        """
        self._transform_manager.get_transform(target_frame_name, source_frame_name)

    def step(self, rgb_img, robot_joints, verbose=False):
        """
        Args:
           rgb_img (np.array): 
           robot_joints (np.array): 
        """
        raise NotImplementedError

    def _calibrate_function(self,
                            pos_1_list,
                            rot_1_list,
                            pos_2_list,
                            rot_2_list,
                            calibration_method="tsai",
                            verbose=True):
        method = None
        if calibration_method == "tsai":
            method = cv2.CALIB_HAND_EYE_TSAI
        elif calibration_method == "horaud":
            method = cv2.CALIB_HAND_EYE_HORAUD

        rot, pos = cv2.calibrateHandEye(
            rot_1_list,
            pos_1_list,
            rot_2_list,
            pos_2_list,
            method=method
        )

        if verbose:
            # print("Rotation matrix: ", rot)
            print("Axis Angle: ", T.quat2axisangle(T.mat2quat(rot)))
            print("Quaternion: ", T.mat2quat(rot))
            print("Translation: ", pos.transpose())
        return (pos, rot)

    def calibrate(self,
                  calibration_method="tsai",
                  verbose=True):
        raise NotImplementedError

    def save_calibration(self, pos, rot, config_folder, json_file_name):
        with open(
            os.path.join(
                config_folder,
                json_file_name,
                # f"camera_{args.camera_id}_{args.camera_type}_{args.post_fix}extrinsics.json",
            ),
            "w",
        ) as f:
            extrinsics = {"translation": pos.tolist(), "rotation": rot.tolist()}
            json.dump(extrinsics, f)
        

class EyeInHandCalibration(HandEyeCalibrationBase):
    def __ini__(self, *args, **kwargs):
        super().__init__(self, *args, **kwargs)
        self.rot_gripper2base_list = []
        self.pos_gripper2base_list = []

    def reset(self):
        super().reset()
        self.rot_gripper2base_list = []
        self.pos_gripper2base_list = []

    def step(self, rgb_img, robot_joints, verbose=False):
        result = self.detect_marker(rgb_img)
        if result is None:
            if verbose:
                print(f"Detection result does not comply with calibration method, discarding this observation")
            return None
        (marker_pos_in_cam, marker_rot_in_cam) = result
        if verbose:
            print(f"Detection result: {result}")

        (gripper_pos_in_base, gripper_rot_in_base) = self.get_gripper_pose_from_model(robot_joints)

        self.add_transform(f"target_{self.counter}", "cam_view_{self.counter}", gripper_rot_in_base, gripper_pos_in_base)        
        self.add_transform(f"ee_{self.counter}", "base", gripper_rot_in_base, gripper_pos_in_base)
        self.pos_target2cam_list.append(marker_pos_in_cam)
        self.rot_target2cam_list.append(marker_rot_in_cam)

        self.pos_gripper2base_list.append(gripper_pos_in_base[..., np.newaxis])
        self.rot_gripper2base_list.append(gripper_rot_in_base)
        self.counter += 1

    def calibrate(self,
                  calibration_method="tsai",
                  verbose=True):
        (cam_pos_in_gripper, cam_rot_in_gripper) = self._calibrate_function(self.pos_gripper2base_list,
                                                                      self.rot_gripper2base_list,
                                                                      self.pos_target2cam_list,
                                                                      self.rot_target2cam_list,
                                                                      calibration_method=calibration_method,
                                                                      verbose=verbose)
        for i in range(self.counter):
            self.add_transform(f"cam_view_{self.counter}", f"ee_{self.counter}", cam_rot_in_gripper, cam_pos_in_gripper)
        return (cam_pos_in_gripper, cam_rot_in_gripper)

class EyeToHandCalibration(HandEyeCalibrationBase):
    def __ini__(self, *args, **kwargs):
        super().__init__(self, *args, **kwargs)
        self.rot_base2gripper_list = []
        self.pos_base2gripper_list = []

    def reset(self):
        super().reset()
        self.rot_base2gripper_list = []
        self.pos_base2gripper_list = []

    def step(self, rgb_img, robot_joints, verbose=False):
        result = self.detect_marker(rgb_img)
        if result is None:
            return None
        (marker_pos_in_cam, marker_rot_in_cam) = result
        if verbose:
            print(f"Detection result: {result}")

        (gripper_pos_in_base, gripper_rot_in_base) = self.get_gripper_pose_from_model(robot_joints)

        self.add_transform(f"target_{self.counter}", "cam_view_{self.counter}", gripper_rot_in_base, gripper_pos_in_base)        
        self.add_transform(f"ee_{self.counter}", "base", gripper_rot_in_base, gripper_pos_in_base)

        base_transformation_in_gripper = self._transform_manager.get_transform("base", f"ee_{self.counter}")
        base_pos_in_gripper = base_transformation_in_gripper[:3, -1:]
        base_rot_in_gripper = base_transformation_in_gripper[:3, :3]

        # print(base_rot_in_gripper, gripper_rot_in_base.transpose())
        # print("=============")
        # print(base_pos_in_gripper, -gripper_rot_in_base.transpose() @ gripper_pos_in_base[:, np.newaxis])
        # print("=============")
        
        # assert(gripper_pos_in_base.shape == base_pos_in_gripper.shape)
        self.pos_target2cam_list.append(marker_pos_in_cam)
        self.rot_target2cam_list.append(marker_rot_in_cam)

        self.pos_base2gripper_list.append(base_pos_in_gripper)
        self.rot_base2gripper_list.append(base_rot_in_gripper)
        self.counter += 1

    def calibrate(self,
                  calibration_method="tsai",
                  verbose=True):
        (cam_pos_in_base, cam_rot_in_base) = self._calibrate_function(self.pos_base2gripper_list,
                                                                      self.rot_base2gripper_list,
                                                                      self.pos_target2cam_list,
                                                                      self.rot_target2cam_list,
                                                                      calibration_method=calibration_method,
                                                                      verbose=verbose)
        return (cam_pos_in_base, cam_rot_in_base)
