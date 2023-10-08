import os
import pybullet as p
import pybullet_data


class PandaURDFModel:
    def __init__(self, asset_folder, use_visualizer=False):
        p_mode = p.GUI
        if use_visualizer is False:
            p_mode = p.DIRECT
        self.physicsClient = p.connect(p.DIRECT)  # or p.DIRECT for non-graphical version
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self.robot_uid = p.loadURDF(
            os.path.join(asset_folder, "assets/franka_panda/franka_panda_arm_marker.urdf"),
            [0.0, 0.0, 0.0],
            p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
        )

    def get_marker_pose(self, joints):

        num_joints = p.getNumJoints(self.robot_uid)

        marker_joint = num_joints - 1
        marker_link = marker_joint

        for i in range(7):
            p.resetJointState(bodyUniqueId=self.robot_uid, jointIndex=i, targetValue=joints[i])

        for i in range(9, 11):
            p.resetJointState(bodyUniqueId=self.robot_uid, jointIndex=i, targetValue=0.04)

        marker_link_pose = p.getLinkState(bodyUniqueId=self.robot_uid, linkIndex=marker_link)
        # print(marker_link_pose)
        return marker_link_pose

    def get_gripper_pose(self, joints):

        num_joints = p.getNumJoints(self.robot_uid)

        marker_joint = num_joints - 1 - 2
        marker_link = marker_joint

        for i in range(7):
            p.resetJointState(bodyUniqueId=self.robot_uid, jointIndex=i, targetValue=joints[i])

        for i in range(9, 11):
            p.resetJointState(bodyUniqueId=self.robot_uid, jointIndex=i, targetValue=0.04)

        marker_link_pose = p.getLinkState(bodyUniqueId=self.robot_uid, linkIndex=marker_link)
        # print(marker_link_pose)
        return marker_link_pose
