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

    # TODO: fix the [ERROR] [1682429626.398549]: Received ACTION_ABORT notification problem

    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils):
        super().__init__(arm, transform_utils)
        self.arm = arm
        self.fm = ForceMeasurmement()
        self.tf_utils = transform_utils
        self.listener = tf.TransformListener()

        self.multimeter_poses = rospy.get_param("~multimeter_poses")
        self.joint_angles = rospy.get_param("~joint_angles")
        self.byod_poses = rospy.get_param("~byod_poses")
        self.power_button_poses = rospy.get_param("~power_button_poses")

        self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
        print("BYOD Action Initialized")
    
    def pre_perceive(self) -> bool:
        print ("in pre perceive")

        success = self.arm.execute_gripper_command(0.0)
        if not success:
            return False
        success = self.arm.execute_gripper_command(1.0)
        if not success:
            return False
        # success = self.arm.execute_gripper_command(0.50)
        # if not success:
        #     return False

        return True

    def act(self) -> bool:

        print ("in act")

        rospy.loginfo(">> Executing BYOD Trajectory <<")
        success = self.get_poses_and_follow_trajactory()
        if not success:
            return False

        rospy.loginfo(">> Pressing Power Button <<")
        success = self.press_power_button(target_status="on")
        if not success:
            return False

        
        rospy.loginfo(">> Rotate the Dial <<")
        success = self.rotate_dial()
        if not success:
            return False
                
        rospy.loginfo(">> Reading multimeter screen <<")
        success = self.read_multimeter_screen()
        if not success:
            return False
        
        return True

    def verify(self) -> bool:
        print ("in verify")
        return True

    def get_poses_and_follow_trajactory(self):

        # TODO: button press poses are also recorded in byod_poses, so we need to remove them from the list

        pose = self.byod_poses
        pose_list = []
        for i in pose.values():
            pose_list.append(get_kinovapose_from_list(list(i.values())))

        #Go byod_pose in joint angles 
        success = self.arm.send_joint_angles(self.joint_angles["byod_safe_pose"])
        if not success:
            return False

        for idx, i in enumerate(pose_list): # travel all the recorded poses

            if idx+1 == 5 or idx+1 == 13:
                success = self.insert_probe(i)
                if not success:
                    return False
                
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

        return True

    def press_power_button(self, target_status: str):

        """
        switch the button on/off
        Args:
            target_status (str): target status of the button, either "on" or "off"
        """

        success = True
        # switch on the device

        pose = get_kinovapose_from_list(list(self.power_button_poses['button_'+target_status+'_up'].values()))
        success = self.arm.send_cartesian_pose(pose)
        if not success:
            return False
        approach_pose = get_kinovapose_from_list(list(self.power_button_poses['button_'+target_status+'_down'].values()))
        approach_pose.z += 0.03
        success = self.arm.send_cartesian_pose(approach_pose)
        if not success:
            return False
        rospy.sleep(0.5)
        success = self.arm.move_down_with_caution(force_threshold=[5,5,8], tool_z_thresh=0.080, velocity= 0.01, retract=True, retract_dist=0.05) # neg because arm is moving in -y axis
        if not success:
            return False
        return True

    def insert_probe(self,pose):

        pose.z += 0.03

        success = self.arm.send_cartesian_pose(pose)
        if not success:
            return False
        rospy.sleep(0.5)

        rospy.loginfo("Moving down with caution")
        success = self.arm.move_down_with_caution(force_threshold=[4,4,4], tool_z_thresh=0.060, velocity= -0.01, approach_axis="y", retract=False) # neg because arm is moving in -y axis 
        if not success:
            return False
        rospy.sleep(0.5)
        rospy.loginfo(">> probe reached<<")
        success = self.arm.execute_gripper_command(0.60)
        if not success:
            return False
        rospy.loginfo(">>Opened Gripper<<")
        return True

    def rotate_dial(self):

        #Go byod_pose in joint angles 
        success = self.arm.send_joint_angles(self.joint_angles["multimeter_pre_pose"])
        if not success:
            return False
        rospy.loginfo(">> Safe Pose Reached <<")

        pose = self.multimeter_poses

        dial_align_pose = pose["dial_align_pose"]
        dial_align_pose = get_kinovapose_from_list(list(dial_align_pose))
        dial_align_pose.z -= 0.02
        success = self.arm.send_cartesian_pose(dial_align_pose)
        if not success:
            return False
        rospy.sleep(1)

        # open gripper
        success = self.arm.execute_gripper_command(0.50)
        if not success:
            return False

        # go down
        success = self.arm.move_down_with_caution(force_threshold=[3,3,3], tool_z_thresh=0.045, velocity=0.008,retract=True, retract_dist=0.006)
        if not success:
            return False
        success = self.arm.stop_arm_velocity()
        if not success:
            return False
        
        rospy.loginfo(">> Dial reached <<")
        
        # hold the dial
        success = self.arm.execute_gripper_command(1.0)
        if not success:
            return False
        rospy.sleep(1)

        rospy.loginfo(">> Dial held <<")

        # rotate the dial with cartesian 
        current_pose = self.arm.get_current_pose()
        current_pose.theta_z_deg -= 21
        success = self.arm.send_cartesian_pose(current_pose)
        if not success:
            return False
        rospy.sleep(1)

        # release the dial
        success = self.arm.execute_gripper_command(0.60)
        if not success:
            return False
        rospy.sleep(1)

        rospy.loginfo(">> Dial released <<")

        # go up
        current_pose = self.arm.get_current_pose()
        current_pose.z += 0.05
        success = self.arm.send_cartesian_pose(current_pose)
        if not success:
            return False
        rospy.sleep(1)

        rospy.loginfo(">> Dial rotated successfully<<")

        return True

    def read_multimeter_screen(self):

        pose = self.multimeter_poses   

        # press red button
        rospy.loginfo(">> Pressing red button <<")
        red_button_pose = pose["red_button_press"]
        red_button_pose = get_kinovapose_from_list(list(red_button_pose))
        red_button_pose.z += 0.01
        success = self.arm.send_cartesian_pose(red_button_pose)
        if not success:
            return False
        rospy.sleep(1)

            # close gripper 60% 
        success = self.arm.execute_gripper_command(0.60)
        if not success:
            return False

            # approach and press button 
        success = self.arm.move_down_with_caution(force_threshold=[4.0,4.0,5.0], tool_z_thresh=0.045, velocity=0.008,retract=True, retract_dist=0.015)
        if not success:
            return False
        
        success = self.arm.stop_arm_velocity()
        if not success:
            return False
        rospy.loginfo(">> Red button pressed <<")


        # press white button
        rospy.loginfo(">> Pressing white button <<")
        white_button_pose = pose["white_button_press"]
        white_button_pose = get_kinovapose_from_list(list(white_button_pose))
        white_button_pose.z += 0.01
        success = self.arm.send_cartesian_pose(white_button_pose)
        if not success:
            return False
        rospy.sleep(1)

            # close gripper 30% 
        success = self.arm.execute_gripper_command(0.60)
        if not success:
            return False

            # approach and press button 
        success = self.arm.move_down_with_caution(force_threshold=[4.0,4.0,5.0], tool_z_thresh=0.045, velocity=0.008,retract=True, retract_dist=0.015)
        if not success:
            return False
        
        success = self.arm.stop_arm_velocity()
        if not success:
            return False
        rospy.loginfo(">> White button pressed <<")


        # Read multimeter screen
        screen_read_pose = pose["screen_read_pose"]
        screen_read_pose = get_kinovapose_from_list(list(screen_read_pose))

        success = self.arm.send_cartesian_pose(screen_read_pose)
        if not success:
            return False
        rospy.sleep(1)

        rospy.loginfo(">> Reading multimeter screen <<")
        # read the screen and publish the value
        # TODO: implement the screen reading and publishing the value
    
        return True


