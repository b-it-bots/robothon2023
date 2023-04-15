#!/usr/bin/env python3

import rospy
from robothon2023.abstract_action import AbstractAction
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from utils.kinova_pose import get_kinovapose_from_list

from kortex_driver.srv import *
from kortex_driver.msg import *

class WindCableAction(AbstractAction):
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:
        super().__init__(arm, transform_utils)
        self.debug = rospy.get_param("~debug", False)

    def pre_perceive(self) -> bool:
        print ("in pre perceive")        
        
        return True

    def act(self) -> bool:
        success = self.wind_cable()
        
        return success

    def verify(self) -> bool:
        print ("in verify")
        return True
    
    def wind_cable(self) -> bool:
        
        # get poses p1 to p8 from the parameter server wind_cable_poses (list of lists) with for loop
        wind_cable_kinova_poses = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses/p" + str(i))
            wind_cable_kinova_poses.append(get_kinovapose_from_list(pose))

        success = self.arm.send_cartesian_pose(wind_cable_kinova_poses[0])

        # close gripper
        self.arm.execute_gripper_command(0.8)

        success = self.arm.traverse_waypoints(wind_cable_kinova_poses)

        wind_cable_kinova_poses2 = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses2/p" + str(i))
            wind_cable_kinova_poses2.append(get_kinovapose_from_list(pose))

        success = self.arm.traverse_waypoints(wind_cable_kinova_poses2)

        return success