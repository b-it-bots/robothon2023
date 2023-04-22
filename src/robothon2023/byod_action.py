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
from utils.kinova_pose import KinovaPose, get_kinovapose_from_pose_stamped
from utils.force_measure import ForceMeasurmement

class SliderAction(AbstractAction):

    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils):
        super().__init__(arm, transform_utils)
        self.arm = arm
        self.fm = ForceMeasurmement()
        self.tf_utils = transform_utils
        self.listener = tf.TransformListener()
        self.slider_pose = PoseStamped()
        
        self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
        print("BYOD Action Initialized")
        

    def pre_perceive(self) -> bool:
        print ("in pre perceive")

        print("Getting slider pose")
        self.get_slider_pose()
        return True

    def act(self) -> bool:
        print ("in act")

        rospy.loginfo(">> Moving arm to slider <<")
        if not self.move_arm_to_slider():
            return False



    def verify(self) -> bool:
        print ("in verify")
        return True

    def do(self) -> bool:

        success = True
        
        success &= self.pre_perceive()
        success &= self.act()
        success &= self.verify()

        return success


    def move_arm_to_slider(self):
        """
        Move arm to along slider in with velocity vector
        """

        offset = 0.08
        rospy.loginfo("slider pose below")
        print(self.slider_pose)
        slider_pose = self.rotate_Z_down(self.slider_pose)
        pose_to_send = get_kinovapose_from_pose_stamped(slider_pose)
        pose_to_send.z += offset # add 10 cm to the z axis and then approach the slider

        success = self.arm.send_cartesian_pose(pose_to_send)
        if not success:
            return False
        return True

    def stop_arm(self):
        """
        Stop arm by sending zero velocity
        """

        velocity_vector = TwistCommand()
        velocity_vector.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED # for proper joypad control
        self.cartesian_velocity_pub.publish(velocity_vector)
        return True

    def get_kinova_pose(self,pose_name):

        pose = rospy.get_param("~"+ pose_name)

        sample = PoseStamped()
        sample.header.stamp = rospy.Time(0)
        sample.header.frame_id = "board_link"
        sample.pose.position.x = pose['pose']['position']['x']
        sample.pose.position.y = pose['pose']['position']['y']
        sample.pose.position.z = pose['pose']['position']['z']
        sample.pose.orientation.x = pose['pose']['orientation']['x']
        sample.pose.orientation.y = pose['pose']['orientation']['y']
        sample.pose.orientation.z = pose['pose']['orientation']['z']
        sample.pose.orientation.w = pose['pose']['orientation']['w']

        pose_in_base_link = self.transform_utils.transformed_pose_with_retries(sample, "base_link", 3)

        pose_in_kinova_pose = get_kinovapose_from_pose_stamped(pose_in_base_link)

        return pose_in_kinova_pose

    def get_trajactory_poses(self,num_poses):
        """
        get poses

        input: num_poses: number of poses to get
        return: list of poses
        """

        pose_list = []
        for i in range(num_poses):
            pose_name = "pose" + str(i+1)
            pose = self.get_kinova_pose(pose_name)
            pose_list.append(pose)
        return pose_list
























