#!/usr/bin/env python3
from __future__ import print_function

import tf
import sys
import rospy
import copy
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.slider_action import SliderAction
import kortex_driver.msg
import numpy as np
from kortex_driver.srv import *
from kortex_driver.msg import *
import actionlib

class SliderTest(object):

    def __init__(self):
        self.fam = FullArmMovement()
        self.slider_action = SliderAction(self.fam)

    def test(self):
        self.slider_action.do()

if __name__ == "__main__":

    rospy.init_node('Slider Test')
    slider_test = SliderTest()
    slider_test.test()
    rospy.spin()

