#!/usr/bin/env python3

import rospy
from robothon2023.abstract_action import AbstractAction
from geometry_msgs.msg import PoseStamped, Quaternion
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from utils.kinova_pose import KinovaPose, get_kinovapose_from_list, get_kinovapose_from_pose_stamped

from kortex_driver.srv import *
from kortex_driver.msg import *

import tf
import math

class ProbeAction(AbstractAction):
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:
        super().__init__(arm, transform_utils)


    def pre_perceive(self) -> bool:
        print ("in pre perceive")        
        
        return True

    def act(self) -> bool:
        success = self.place_probe_in_holder()
        
        return success

    def verify(self) -> bool:
        print ("in verify")
        return True
    
    def place_probe_in_holder(self):
        '''
        Place the probe in the holder
        '''

        # go the probe initial position

        # get the probe initial position from tf
        msg = PoseStamped()
        msg.header.frame_id = "probe_initial_link"
        msg.header.stamp = rospy.Time(0)
        probe_initial_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

        # convert the probe initial position to a kinova pose
        probe_initial_pose = get_kinovapose_from_pose_stamped(probe_initial_pose)

        # send the probe initial position to the arm
        success = self.arm.send_cartesian_pose(probe_initial_pose)

        if not success:
            rospy.logerr("Failed to move to the probe initial position")
            return False
        
        # close the gripper
        success = self.arm.execute_gripper_command(0.7)

        if not success:
            rospy.logerr("Failed to close the gripper")
            return False
        
        # remove the probe from the box
        probe_initial_pose_in_bl = self.transform_utils.get_pose_from_link("probe_initial_link", "board_link")
        probe_initial_pose_in_bl.pose.position.x -= 0.05

        # convert the pose to base_link
        probe_initial_pose_in_bl = self.transform_utils.transformed_pose_with_retries(probe_initial_pose_in_bl, "base_link")
        
        # convert the probe initial position to a kinova pose
        move_back_pose = get_kinovapose_from_pose_stamped(probe_initial_pose_in_bl)
        success = self.arm.send_cartesian_pose(move_back_pose)

        if not success:
            rospy.logerr("Failed to move back the probe")
            return False

        # move up a bit by 10cm
        probe_initial_pose_in_bl.pose.position.z += 0.1
        move_up_pose = get_kinovapose_from_pose_stamped(probe_initial_pose_in_bl)
        success = self.arm.send_cartesian_pose(move_up_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False

        # get the probe holder position from rosparam
        probe_holder_pose = rospy.get_param("~probe_holder_pose")

        # convert the probe holder position to a kinova pose
        probe_holder_pose = get_kinovapose_from_list(probe_holder_pose)

        success = self.arm.send_cartesian_pose(probe_holder_pose)

        if not success:
            rospy.logerr("Failed to move to the probe holder position")
            return False
        
        # open the gripper
        success = self.arm.execute_gripper_command(0.0)

        if not success:
            rospy.logerr("Failed to open the gripper")
            return False
        
        # move up a bit by 20cm
        probe_holder_pose.z += 0.2
        
        success = self.arm.send_cartesian_pose(probe_holder_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        


        
        return True