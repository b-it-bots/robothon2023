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
import numpy as np

from kortex_driver.srv import *
from kortex_driver.msg import *

from robothon2023.full_arm_movement import FullArmMovement

class ForceMeasurmement:

    def __init__(self, force_threshold: list = [10,10, 10], topic_name: String = "None"):
        self._force_subscriber = rospy.Subscriber("/my_gen3/base_feedback", kortex_driver.msg.BaseCyclic_Feedback, self._force_callback)
        self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
        self._force = {'x': [], 
                       'y': [], 
                       'z': []}

        self._force_threshold = force_threshold 
        self.monitoring = False
        self.force_limit_flag = False

        self.fam = FullArmMovement()

    def _force_callback(self, msg):

        self._force['x'].append(msg.base.tool_external_wrench_force_x)
        self._force['y'].append(msg.base.tool_external_wrench_force_y)
        self._force['z'].append(msg.base.tool_external_wrench_force_z)

        if len(self._force['x']) > 10:
            self._force['x'].pop(0)
            self._force['y'].pop(0)
            self._force['z'].pop(0)
        
            if self.monitoring:
                self.force_check()
                if self.force_limit_flag:
                    self._force['x'].clear()
                    self._force['y'].clear()
                    self._force['z'].clear()
        else:
            # rospy.logwarn("Force data not enough")
            pass

        
  # check if force is greater than threshold in continuous

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

        force_accumulated = [0,0,0]

        force_accumulated[0] = abs(np.mean(self._force['x']) - self._force['x'][-1])
        force_accumulated[1] = abs(np.mean(self._force['y']) - self._force['y'][-1])
        force_accumulated[2] = abs(np.mean(self._force['z']) - self._force['z'][-1])


        yellow = "\033[93m"

        data = [0,0,0]

        # convert list into dict
        force_dict = {}
        idx = ['x', 'y', 'z']
        for i, f in zip(idx,force_accumulated):
            force_dict[i] = f

        # print("*"* 20)
        # print(force_dict)
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

        if data[0] == 1 or data[1] == 1 or data[2] == 1:
            self.set_force_limit_flag()
            # pass
        if force_dict["x"] > 20 or force_dict["y"] > 20 or force_dict["z"] > 20:
            self.fam.apply_E_STOP()

    
    # Returns the force measured by the robot
    def get_force(self):
        return self._force

    def enable_monitoring(self):
        
        self.monitoring = True
        return True
    
    def disable_monitoring(self):

        self.monitoring = False
        return True

    def set_force_limit_flag(self):

        self.force_limit_flag = True
        return True
    
    def reset_force_limit_flag(self):
            
        self.force_limit_flag = False
        return True 


if __name__ == "__main__":
    
    pass
    
    # rospy.init_node("force_monitoring", anonymous=True)
    # fm = ForceMeasurmement()
    # fm.enable_monitoring()
    # rospy.spin()


