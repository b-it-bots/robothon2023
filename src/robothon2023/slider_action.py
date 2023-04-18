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

    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils, fm: ForceMeasurmement):
        super().__init__(arm, transform_utils)
        self.arm = arm
        self.fm = ForceMeasurmement()
        self.tf_utils = transform_utils
        self.listener = tf.TransformListener()
        self.slider_pose = PoseStamped()
        self.get_slider_pose()

        self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)

        self.tooltip_pose_sub = rospy.Subscriber('/my_gen3/base_feedback', PoseStamped, self.tooltip_pose_callback)

        print("Slider Action Initialized")
        

    def pre_perceive(self) -> bool:
        print ("in pre perceive")
        return True

    def act(self) -> bool:
        print ("in act")


        rospy.loginfo(">> Moving arm to slider <<")
        if not self.move_arm_to_slider():
            return False

        rospy.loginfo(">> Clossing gripper <<")
        if not self.arm.execute_gripper_command(0.53    ):
            return False

        rospy.loginfo(">> Approaching slider with caution <<")
        if not self.approach_slider_with_caution():
            return False

        rospy.loginfo(">> Moving arm along the slider <<")
        if not self.move_arm_along_slider():
            return False

        rospy.loginfo(">> Stopping arm <<")
        if not self.stop_arm():
            return False

        rospy.loginfo(">> Moving slider back  <<")
        if not self.move_slider_back():
            return False

        rospy.loginfo(">> Stopping arm <<")
        if not self.stop_arm():
            return False

        # rospy.loginfo(">> open gripper <<")
        # if not self.arm.execute_gripper_command(0.50):
        #     return False

        rospy.loginfo(">> Retract arm back <<")
        if not self.retract_arm_back():
            return False

        rospy.loginfo(">> Stopping arm <<")
        if not self.stop_arm():
            return False

        rospy.loginfo(">> Process finished successfully <<")

        return True

    def verify(self) -> bool:
        print ("in verify")

        ## can be verified by checking the current location of the EEF 
        return True

    def do(self) -> bool:
        return self.act()


    def tooltip_pose_callback(self, msg):
        self.tooltip_pose_z_with_base = msg.base.tool_pose_z


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

    def approach_slider_with_caution(self):
        """
        Approach slider with caution
        """

        offset = 0.05 # offset for the distance from tool frame to the tool tip
        rate_loop = rospy.Rate(10)
        self.fm.set_force_threshold(force=[10,10,4]) # force in z increases to 4N when it is in contact with the board

        # enable force monitoring
        self.fm.enable_monitoring()
        
        # calculate velocity
        distance = offset; time = 7 # move 5 cm in 6 seconds
        velocity = distance/time

        # create twist command to move towards the slider
        approach_twist = TwistCommand()
        approach_twist.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
        approach_twist.twist.linear_z = velocity

        while not self.fm.force_limit_flag and not rospy.is_shutdown(): 
            # check for force limit flag and stop if it is true
            # check for the z axis of the tooltip and stop if it is less than 0.111(m) (the z axis of the slider) from base frame

            if self.fm.force_limit_flag:
                break
            self.cartesian_velocity_pub.publish(approach_twist)
            rate_loop.sleep()

        success = self.stop_arm()
        if not success:
            return False
        
        self.fm.disable_monitoring()

        distance = 0.008 ; time = 1 # move back 8 mm
        velocity = distance/time

        retract_twist = TwistCommand()
        retract_twist.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
        retract_twist.twist.linear_z = -velocity
        if self.fm.force_limit_flag:
            self.cartesian_velocity_pub.publish(retract_twist)
            rospy.sleep(time)
        
        self.stop_arm()
        return True


    def move_arm_along_slider(self):
        """
        Move arm along slider in with velocity vector
        """

        distance = 0.042 # m  (0.045 is the length the slider can travel and 0.005 is a small offset for safety reasons)
        time = 3 # s
        slider_velocity = round(distance/time , 3) # m/s

        slider_velocity_cmd = TwistCommand()
        slider_velocity_cmd.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
        slider_velocity_cmd.twist.linear_x = slider_velocity

        self.cartesian_velocity_pub.publish(slider_velocity_cmd)
        rospy.sleep(time)

        return True

    def move_slider_back(self):
        """
        Move arm back
        """
        distance = 0.05 # m  (0.045 is the length the slider can travel and 0.005 is a small offset for safety reasons)
        time = 3 # s
        slider_velocity = round(distance/time , 3) # m/s
        
        slider_velocity_cmd = TwistCommand()
        slider_velocity_cmd.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
        slider_velocity_cmd.twist.linear_x = -slider_velocity

        self.cartesian_velocity_pub.publish(slider_velocity_cmd)
        rospy.sleep(time)

        return True
    
    def retract_arm_back(self):
        """
        Move arm back using twist command
        """
        retract_velocity = 0.02 # m/s
        retract_twist_cmd = TwistCommand()
        retract_twist_cmd.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
        retract_twist_cmd.twist.linear_z = -retract_velocity # neg velocity for upwards movement
        self.cartesian_velocity_pub.publish(retract_twist_cmd)
        rospy.sleep(3.0)
        return True

    def stop_arm(self):
        """
        Stop arm by sending zero velocity
        """

        velocity_vector = TwistCommand()
        velocity_vector.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED # for proper joypad control
        self.cartesian_velocity_pub.publish(velocity_vector)
        return True

    def get_slider_pose(self):
        """
        Get slider pose
        """
        msg = PoseStamped()
        msg.header.frame_id = "slider_start_link"
        msg.header.stamp = rospy.Time(0)
        self.slider_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

        return True

    def rotate_Z_down(self, msg: PoseStamped) -> PoseStamped:

        msg = msg
        msg.header.stamp = rospy.Time.now()

        # quaternion to euler for given pose
        e = list(tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))

        rot_eluer = [math.pi, 0.0,math.pi]  # rotate 180 degrees around x axis to make the z pointing (down) and the y will be in opposite direction than original and we 
        # add 180 around z axis to set the orientation of camera facing the gui screen 

        e[0] += rot_eluer[0]
        e[1] += rot_eluer[1]
        e[2] += rot_eluer[2]

        q = list(tf.transformations.quaternion_from_euler(e[0], e[1], e[2]))

        msg.pose.orientation = Quaternion(*q)
        msg = self.transform_utils.transformed_pose_with_retries(msg, 'base_link')
        return msg



























