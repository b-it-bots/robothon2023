#!/usr/bin/env python3
from __future__ import print_function

import tf
import sys
import rospy
import copy
from std_msgs.msg import String
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.slider_action import SliderAction
from robothon2023.button_press_action import ButtonPressAction
from robothon2023.transform_utils import TransformUtils
import numpy as np
import actionlib

class RobothonTask(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.joint_angles = rospy.get_param("~joint_angles", None)
        self.fam = FullArmMovement()
        self.tu = TransformUtils()
        self.button_press_action = ButtonPressAction(self.fam, self.tu)

        self.loop_rate = rospy.Rate(10.0)

        self.setup_arm_for_pick()
        #rospy.sleep(3.0)
        rospy.loginfo("READY!")
    

    def setup_arm_for_pick(self):
        """Setup the arm to go to pick pose
        :returns: None

        """
        self.fam.clear_faults()
        self.fam.subscribe_to_a_robot_notification()
        # self.fam.test_send_joint_angles(self.joint_angles["vertical_pose"])
        print (self.joint_angles['perceive_table'])
        self.fam.send_joint_angles(self.joint_angles["perceive_table"])
        self.fam.execute_gripper_command(0.0) #Open the gripper 
        #self.fam.example_send_gripper_command(0.5) #half close the gripper 
        self.fam.execute_gripper_command(0.9) #full close the gripper 
        rospy.sleep(0.1)


    def test(self):
        self.button_press_action.do()
        rospy.sleep(0.1)
        pass

if __name__ == "__main__":
    rospy.init_node('robothon_task')
    task = RobothonTask()
    task.test()
    #PAP.test_go_to_board()
    #PAP.test_press_button()
    rospy.spin()

