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
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', kortex_driver.msg.TwistCommand, queue_size=1)
        self.loop_rate = rospy.Rate(10.0)

    def pre_perceive(self) -> bool:
        print ("in pre perceive")        
        
        return True

    def act(self) -> bool:
        # success = self.place_probe_in_holder()

        success = self.pick_magnet()
        
        return success

    def verify(self) -> bool:
        print ("in verify")
        return True
    
    def place_probe_in_holder(self):
        '''
        Place the probe in the holder
        '''

        # go the probe initial position
        success = self.arm.execute_gripper_command(0.0) # open the gripper

        # get the probe initial position from tf
        msg = PoseStamped()
        msg.header.frame_id = "probe_initial_link"
        msg.header.stamp = rospy.Time(0)
        probe_initial_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True, offset=[0, 0, math.pi/2])

        # convert the probe initial position to a kinova pose
        probe_initial_pose_kp = get_kinovapose_from_pose_stamped(probe_initial_pose)

        # send the probe initial position to the arm
        print("[probe_action] moving to probe initial position")
        success = self.arm.send_cartesian_pose(probe_initial_pose_kp)

        if not success:
            rospy.logerr("[probe_action] Failed to move to the probe initial position")
            return False
        
        print('[probe_action] reached probe initial position')
        
        # close the gripper
        success = self.arm.execute_gripper_command(0.9)

        if not success:
            rospy.logerr("Failed to close the gripper")
            return False
        
        # remove the probe from the box
        msg = PoseStamped()
        msg.header.frame_id = "probe_initial_link"
        msg.header.stamp = rospy.Time(0)
        probe_initial_pose_in_bdl = self.transform_utils.transformed_pose_with_retries(msg, "board_link")

        probe_initial_pose_in_bdl.pose.position.x -= 0.08

        # convert the pose to base_link
        probe_mb_pose_in_bl = self.transform_utils.transformed_pose_with_retries(probe_initial_pose_in_bdl,
                                                                                 "base_link", 
                                                                                 execute_arm=True, 
                                                                                 offset=[0, 0, math.pi/2])

        # convert to a kinova pose
        move_back_pose = get_kinovapose_from_pose_stamped(probe_mb_pose_in_bl)

        print("[probe_action] moving back the probe by 5cm")
        success = self.arm.send_cartesian_pose(move_back_pose)

        if not success:
            rospy.logerr("Failed to move back the probe")
            return False
        
        print("[probe_action] moved back the probe by 5cm")

        # move up a bit
        move_up_pose = move_back_pose
        move_up_pose.x += 0.10
        move_up_pose.y -= 0.30
        move_up_pose.z += 0.25

        print("[probe_action] moving up the probe")
        success = self.arm.send_cartesian_pose(move_up_pose)

        # send joint angles to the arm
        # [0.8197946865095573, 1.7372213912136376, 1.6281319213773762, -0.7847078720428993, 0.06814585646743704, -1.4455723618104983, -3.093051482717028, 0.5578947226687478]
        
        joint_angles = [0.8197946865095573, 1.7372213912136376, 1.6281319213773762,
                        -0.7847078720428993, 0.06814585646743704, -1.4455723618104983, -3.093051482717028, 0.5578947226687478]
        
        # convert the joint angles to degrees
        joint_angles = [math.degrees(ja) for ja in joint_angles]

        # rotate joinnt 7 by 180 degrees
        joint_angles[6] += 180.0
        
        print("[probe_action] sending joint angles")
        success = self.arm.send_joint_angles(joint_angles)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        print("moved up the probe to above the holder")

        # get the current pose of the arm
        current_pose = self.arm.get_current_pose()

        # move down a bit
        current_pose.z -= 0.05

        success = self.arm.send_cartesian_pose(current_pose)

        if not success:
            rospy.logerr("Failed to move down the probe")
            return False
        
        print("moved down the probe to above the holder")
        
        # open the gripper
        success = self.arm.execute_gripper_command(0.0)

        if not success:
            rospy.logerr("Failed to open the gripper")
            return False

        # move up a bit by 20cm
        current_pose.z += 0.2
        
        success = self.arm.send_cartesian_pose(current_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        return True
    
    def pick_magnet(self):
        # get magnet pose from param server
        magnet_pose = rospy.get_param("~magnet_pose")
        magnet_kinova_pose = get_kinovapose_from_list(magnet_pose)

        # rotate the yaw by 180 degrees
        magnet_kinova_pose.theta_z_deg += 180.0

        # send magnet pose to the arm
        print("[probe_action] moving to magnet position")
        success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the magnet position")
            return False
        
        print("[probe_action] reached magnet position")

        # close the gripper
        success = self.arm.execute_gripper_command(0.7)

        if not success:
            rospy.logerr("Failed to close the gripper")
            return False
        
        # move up a bit
        magnet_kinova_pose.z += 0.3

        success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        # go to the door knob position
        # get the door knob position from tf
        msg = PoseStamped()
        msg.header.frame_id = "door_knob_link"
        msg.header.stamp = rospy.Time(0)
        door_knob_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True)

        # convert the door knob pose to a kinova pose
        door_knob_kinova_pose = get_kinovapose_from_pose_stamped(door_knob_pose)

        # move up a bit
        door_knob_kinova_pose.z += 0.028

        # send the door knob pose to the arm
        print("[probe_action] moving to door knob position")
        success = self.arm.send_cartesian_pose(door_knob_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the door knob position")
            return False
        
        print("[probe_action] reached door knob position")

        linear_vel_z = 0.0025

        print("[probe_action] moving up with linear velocity: {}".format(linear_vel_z))
        msg = kortex_driver.msg.TwistCommand()
        msg.twist.linear_z = linear_vel_z
        self.cart_vel_pub.publish(msg)
        rospy.sleep(5)
        msg.twist.linear_z = 0.0
        self.cart_vel_pub.publish(msg)
    
        # get current pose of the arm
        current_pose = self.arm.get_current_pose()

        # go up by 5cm
        current_pose.z += 0.1

        success = self.arm.send_cartesian_pose(current_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        # go to the magnet position
        print("[probe_action] moving to magnet position")
        magnet_pose = rospy.get_param("~magnet_pose")
        magnet_kinova_pose = get_kinovapose_from_list(magnet_pose)
        success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the magnet position")
            return False

        # open the gripper
        success = self.arm.execute_gripper_command(0.0)

        if not success:
            rospy.logerr("Failed to open the gripper")
            return False
        
        # move up a bit
        magnet_kinova_pose.z += 0.3

        success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False

        print("[probe_action] target reached")
        return True