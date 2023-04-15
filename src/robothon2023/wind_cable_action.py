#!/usr/bin/env python3

import rospy
from robothon2023.abstract_action import AbstractAction
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from utils.kinova_pose import get_kinovapose_from_list, get_kinovapose_from_pose_stamped
from geometry_msgs.msg import PoseStamped, Quaternion
from kortex_driver.srv import *
from kortex_driver.msg import *
import tf
import os

class WindCableAction(AbstractAction):
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:
        super().__init__(arm, transform_utils)
        self.debug = rospy.get_param("~debug", False)

    def pre_perceive(self) -> bool:
        print ("in pre perceive")        
        
        return True

    def act(self) -> bool:
        # TODO: run the below method to convert the poses to board_link frame and write to a yaml file
        # it might need debugging
        success = self.transform_poses()

        # success = self.wind_cable()

        return success

    def verify(self) -> bool:
        print ("in verify")
        return True
    
    def transform_poses(self):

        wind_cable_poses = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses/p" + str(i))
            kp = get_kinovapose_from_list(pose)
            pose = kp.to_pose_stamped()
            pose_in_board = self.transform_utils.transformed_pose_with_retries(pose, "board_link")
            wind_cable_poses.append(pose_in_board)
        
        wind_cable_poses2 = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses2/p" + str(i))
            kp = get_kinovapose_from_list(pose)
            pose = kp.to_pose_stamped()
            pose_in_board = self.transform_utils.transformed_pose_with_retries(pose, "board_link")
            wind_cable_poses2.append(pose_in_board)

        print('length of wind_cable_poses: ', len(wind_cable_poses))
        print('length of wind_cable_poses2: ', len(wind_cable_poses2))

        # get relative path to the config folder
        config_path = os.path.join(os.path.dirname(__file__), '../..', 'config')

        # write the poses to a yaml file
        with open(config_path+"/wind_cable_poses.yaml", "w+") as f:
            f.write("wind_cable_poses:\n")
            for i in range(1, 11):
                print('i: ', i)
                p: PoseStamped = wind_cable_poses[i]
                # convert orientation to euler angles using tf
                euler = tf.transformations.euler_from_quaternion([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])
                f.write(f"  p{str(i)} : [{p.pose.position.x}, {p.pose.position.y}, {p.pose.position.z}, {euler[0]}, {euler[1]}, {euler[2]}]\n")
            f.write("wind_cable_poses2:\n")
            for i in range(1, 11):
                print('i: ', i)
                p: PoseStamped = wind_cable_poses2[i]
                # convert orientation to euler angles using tf
                euler = tf.transformations.euler_from_quaternion([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])
                f.write(f"  p{str(i)} : [{p.pose.position.x}, {p.pose.position.y}, {p.pose.position.z}, {euler[0]}, {euler[1]}, {euler[2]}]\n")

        return True
    
    def wind_cable(self) -> bool:
        
        wind_cable_kinova_poses = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses/p" + str(i))

            # convert the euler angles to quaternion
            euler = [pose[3], pose[4], pose[5]]
            quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

            msg = PoseStamped()
            msg.header.frame_id = "board_link"
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            msg.pose.orientation = Quaternion(*quaternion)

            # convert to base_link frame
            msg_in_base = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

            # convert to kinova_pose
            kp = get_kinovapose_from_pose_stamped(msg_in_base)

            wind_cable_kinova_poses.append(kp)

        success = self.arm.send_cartesian_pose(wind_cable_kinova_poses[0])

        # close gripper
        self.arm.execute_gripper_command(0.8)

        success = self.arm.traverse_waypoints(wind_cable_kinova_poses)

        wind_cable_kinova_poses2 = []
        for i in range(1, 11):
            # convert the euler angles to quaternion
            euler = [pose[3], pose[4], pose[5]]
            quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

            msg = PoseStamped()
            msg.header.frame_id = "board_link"
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            msg.pose.orientation = Quaternion(*quaternion)

            # convert to base_link frame
            msg_in_base = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

            # convert to kinova_pose
            kp = get_kinovapose_from_pose_stamped(msg_in_base)

            wind_cable_kinova_poses.append(kp)

        success = self.arm.traverse_waypoints(wind_cable_kinova_poses2)

        # go out of the way to keep the probe safe
        # TODO: figure out which way to go 
        

        return success