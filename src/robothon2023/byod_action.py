#!/usr/bin/env python3
import tf
import rospy
import numpy as np
import math 
from kortex_driver.msg import TwistCommand, CartesianReferenceFrame

from robothon2023.abstract_action import AbstractAction
from robothon2023.full_arm_movement import FullArmMovement
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3
from robothon2023.transform_utils import TransformUtils
from utils.kinova_pose import KinovaPose, get_kinovapose_from_pose_stamped, get_kinovapose_from_list
from utils.force_measure import ForceMeasurmement

class ByodAction(AbstractAction):

    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils):
        super().__init__(arm, transform_utils)
        self.arm = arm
        self.fm = ForceMeasurmement()
        self.tf_utils = transform_utils
        self.listener = tf.TransformListener()

        self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
        print("BYOD Action Initialized")
        

    def pre_perceive(self) -> bool:
        print ("in pre perceive")
        return True


    def act(self) -> bool:

        print ("in act")

        rospy.loginfo(">> Moving arm to slider <<")
        success = self.get_poses_and_follow_trajactory()
        if not success:
            return False
        return True

    def verify(self) -> bool:
        print ("in verify")
        return True

    def get_poses_and_follow_trajactory(self):

        pose = rospy.get_param("~byod_poses")
        pose_list = []
        for i in pose.values():
            pose_list.append(get_kinovapose_from_list(list(i.values())))

        #Go byod_pose in joint angles 

        

        while not rospy.is_shutdown():
            for idx, i in enumerate(pose_list):

                if idx+1 == 17 or idx+1 == 19:
                    # TODO:implement force based button push
                    i.z += 0.05
                    success = self.arm.send_cartesian_pose(i)
                    if not success:
                        return False
                    rospy.sleep(1)
                    self.arm.move_down_with_caution(force_threshold=[5,5,5], tool_z_thresh=0.079, velocity=0.01)
                    continue

                if idx+1 == 5 or idx+1 == 13:
                    # TODO : implement force based placing of probe
                    i.z += 0.03
                    success = self.arm.send_cartesian_pose(i)
                    if not success:
                        return False
                    rospy.sleep(1)
                    self.arm.move_down_with_caution(force_threshold=[4,4,4], tool_z_thresh=0.060, velocity= -0.01, approach_axis="y", retract=False) # neg because arm is moving in -y axis 
                    
                    rospy.sleep(1)
                    success = self.arm.execute_gripper_command(0.60)
                    if not success:
                        return False
                    rospy.loginfo(">> Opened Gripper <<")
                    continue

                success = self.arm.send_cartesian_pose(i)
                if not success:
                    return False
                rospy.sleep(1)
                rospy.loginfo(">> pose_"+str(idx+1)+" reached<<")

                list_OG = [1,5,13] 
                list_CG = [2,8,15]
                if idx+1 in list_OG:
                    success = self.arm.execute_gripper_command(0.60)
                    # rospy.sleep(1)
                    if not success:
                        return False
                    rospy.loginfo(">> Opened Gripper <<")

                if idx+1 in list_CG:
                    success = self.arm.execute_gripper_command(0.95)
                    # rospy.sleep(1)
                    if not success:
                        return False
                    rospy.loginfo(">> Closed Gripper <<")

                if idx == len(pose_list)-1 or rospy.is_shutdown():
                    break
        return True

    def read_multimeter_screen(self):

        pose = rospy.get_param("~multimeter_poses")    

        pose_list = []
        for i in pose.values():
            pose_list.append(get_kinovapose_from_list(list(i.values())))
        success = self.arm.send_cartesian_pose(pose_list[1]) # MULTIMETER POSE to read the screen 
        if not success:
            return False
        
        return True

    def rotate_dial(self):

        pose = rospy.get_param("~multimeter_poses")    

        pose_list = []
        for i in pose.values():
            pose_list.append(get_kinovapose_from_list(list(i.values())))

        success = self.arm.send_cartesian_pose(pose_list[0]) # MULTIMETER POSE above the dial
        if not success:
            return False
        rospy.sleep(1)
        rospy.loginfo(">> multimeter reached<<")















