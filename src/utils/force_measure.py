#!/usr/bin/env python3
"""
Author: Ravisankar Selvaraju
Date: 31.03.2021
Mail id :  ravisankar1223@gmail.com

"""

import sys
import rospy
import time
import math
import tf
from std_msgs.msg import Int16MultiArray , String
from kortex_driver.msg import TwistCommand
import kortex_driver

from kortex_driver.srv import *
from kortex_driver.msg import *

from robothon2023.full_arm_movement import FullArmMovement

class ForceMeasurmement:

    def __init__(self, force_threshold: list = [10, 10, 10], topic_name: String = "None"):
        self._force_subscriber = rospy.Subscriber("/my_gen3/base_feedback", kortex_driver.msg.BaseCyclic_Feedback, self._force_callback)
        self.pub_force_status = rospy.Publisher('/my_gen3/end_effector_force_status'+'/'+topic_name, Int16MultiArray, queue_size=5)
        self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
        self._force = None
        self._force_threshold = force_threshold 

        self.fam = FullArmMovement()

    def _force_callback(self, msg):
        self._force = [msg.base.tool_external_wrench_force_x, 
                       msg.base.tool_external_wrench_force_y,
                       msg.base.tool_external_wrench_force_z]
        
        self.force_check() # check if force is greater than threshold in continuous

    def set_force_threshold(self, force):
        """
        Sets the force threshold
        Args:
            force (list): list of 3 floats, representing the force threshold in x, y, z directions
        """
        self._force_threshold = list(force)

    def force_check(self):
        """
        Checks if the force is greater than the threshold
        By default, the threshold is set to 10 N in all directions
        use set_force_threshold to change the threshold
        Returns:
            bool: 1, if force is greater than threshold, 0 otherwise
        """

        force_limit = self._force_threshold

        # rospy.loginfo("Current force: {}".format(current_force))

        # bool_array = Int16MultiArray()
        yellow = "\033[93m"

        data = [0,0,0]

        # convert list into dict
        force_dict = {}
        idx = ['x', 'y', 'z']
        for i, f in zip(idx,self._force):
            force_dict[i] = f

        if force_dict['x'] > force_limit[0]:
            rospy.loginfo(yellow + "Force limit reached in x direction" + "\033[0m")
            rospy.loginfo(force_dict['x'])
            data[0] = 1

        if force_dict['y'] > force_limit[1]:
            rospy.loginfo(yellow + "Force limit reached in y direction" + "\033[0m")
            rospy.loginfo(force_dict['y'])
            data[1] = 1

        if force_dict['z'] >  force_limit[2]:
            rospy.loginfo(yellow + "Force limit reached in z direction" + "\033[0m")
            rospy.loginfo(force_dict['z'])
            data[2] = 1
        
        # bool_array.data = data
        # self.pub_force_status.publish(bool_array)

        if data[0] == 1 or data[1] == 1 or data[2] == 1:
            self.fam.apply_E_STOP()
            # self.fam.clear_faults()
    
    # Returns the force measured by the robot
    def get_force(self):
        return self._force

if __name__ == "__main__":
    pass


