#!/usr/bin/env python3
from __future__ import print_function

import rospy
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from kortex_driver.srv import *
from kortex_driver.msg import *

from robothon2023.wind_cable_action import WindCableAction

class WindCable(object):

    """Probe test using full arm movement"""

    def __init__(self):
        self.fam = FullArmMovement()
        self.transform_utils = TransformUtils()
        self.wind_cable = WindCableAction(self.fam, self.transform_utils)
        self.joint_angles = rospy.get_param("~joint_angles", None)
        self.setup_arm_for_pick()

    def setup_arm_for_pick(self):
        """Setup the arm to go to pick pose
        :returns: None

        """
        self.fam.clear_faults()
        self.fam.subscribe_to_a_robot_notification()
        self.fam.send_joint_angles(self.joint_angles["perceive_table"])
        self.fam.execute_gripper_command(0.35) #Open the gripper

    def test(self):
        if self.wind_cable.do():
            return True


if __name__ == "__main__":
    rospy.init_node('probe_test')
    WC =  WindCable()
    WC.test()