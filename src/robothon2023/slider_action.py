#!/usr/bin/env python3
import tf
import rospy
import numpy as np
import math 
from kortex_driver.msg import TwistCommand

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
        # self.fm = ForceMeasurmement()
        self.tf_utils = transform_utils
        self.listener = tf.TransformListener()
        self.slider_pose = PoseStamped()
        self.slider_pose = rospy.wait_for_message('/task_board_detector/slider_start_pose', PoseStamped)
        # self.slider_pose_sub = rospy.Subscriber('/mcr_perception/object_selector/output/object_pose', PoseStamped, self.slider_pose_callback, queue_size=10)
        self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
        
        print("Slider Action Initialized")
        

    def pre_perceive(self) -> bool:
        print ("in pre perceive")
        return True

    def act(self) -> bool:
        print ("in act")

        # velocity_value = 0.01 # m/s
        distance = 0.045 - 0.005 # m  (0.045 is the length the slider can travel and 0.005 is a small offset for safety reasons)
        time = 3 # s

        velocity_value = round(distance/time , 3) # m/s

        self.slider_velocity_twist = self.create_twist_from_velocity(velocity_value)

        rospy.loginfo(">> Moving arm to slider <<")
        self.move_arm_to_slider()

        rospy.loginfo(">> Clossing gripper <<")
        self.arm.execute_gripper_command(0.85) # close gripper to 85% 

        rospy.loginfo(">> Moving arm along the slider <<")
        self.move_arm_along_slider()
        rospy.sleep(time)

        rospy.loginfo(">> Stopping arm <<")
        self.stop_arm()
        rospy.sleep(1)

        rospy.loginfo(">> Moving slider back  <<")
        self.move_slider_back()
        rospy.sleep(time)

        rospy.loginfo(">> Stopping arm <<")
        self.stop_arm()

        rospy.loginfo(">> Process finished successfully <<")

        return True

    def verify(self) -> bool:
        print ("in verify")

        ## can be verified by checking the current location of the EEF 
        return True

    def do(self) -> bool:
        return self.act()

    
    def slider_pose_callback(self, msg):
        """
        Callback function for slider pose
        """
        print("Slider pose received")

        self.slider_pose = msg.pose


    def create_twist_from_velocity(self, velocity) -> Twist:
        """
        Create Twist message from velocity vector

        input: velocity vector :: np.array
        output: velocity vector :: Twist
        """

        velocity_vector = Twist()
        velocity_vector.linear = Vector3(0,velocity,0)
                                         
        return velocity_vector

    def convert_twist_twistcommand(self, velocity: Twist) -> TwistCommand:

        velocity_cmd = TwistCommand()
        velocity_cmd.twist.linear_x = velocity.linear.x
        velocity_cmd.twist.linear_y = velocity.linear.y
        velocity_cmd.twist.linear_z = velocity.linear.z

        return velocity_cmd
    

    def rotate_Z_down(self, msg: PoseStamped) -> PoseStamped:
        
        msg = msg
        msg.header.stamp = rospy.Time.now()
        q = list(tf.transformations.quaternion_from_euler(math.pi, 0.0, math.pi/2))
        msg.pose.orientation = Quaternion(*q)
        msg.pose.position.z += 0.01
        msg = self.transform_utils.transformed_pose_with_retries(msg, 'base_link')
        return msg

    def move_arm_to_slider(self):
        """
        Move arm to along slider in with velocity vector
        """

        rospy.loginfo("slider pose below")
        print(self.slider_pose)
        slider_pose = self.rotate_Z_down(self.slider_pose)
        pose_to_send = get_kinovapose_from_pose_stamped(slider_pose)
        self.arm.send_cartesian_pose(pose_to_send)
        return True


    def move_arm_along_slider(self):
        """
        Move arm along slider in with velocity vector
        """

        print("twist value below")
        print(self.slider_velocity_twist)
        slider_velocity = self.convert_twist_twistcommand(self.slider_velocity_twist)
        self.cartesian_velocity_pub.publish(slider_velocity)
        return True

    def move_slider_back(self):
        """
        Move arm back
        """

        print("twist value below")
        print(self.slider_velocity_twist)
        self.slider_velocity_twist.linear.y = -self.slider_velocity_twist.linear.y
        slider_velocity = self.convert_twist_twistcommand(self.slider_velocity_twist)
        self.cartesian_velocity_pub.publish(slider_velocity)
        return True

    def stop_arm(self):
        """
        Stop arm by sending zero velocity
        """

        velocity_vector = TwistCommand()
        velocity_vector.twist.linear_x = 0.0
        velocity_vector.twist.linear_y = 0.0
        velocity_vector.twist.linear_z = 0.0
        velocity_vector.twist.angular_x = 0.0
        velocity_vector.twist.angular_y = 0.0
        velocity_vector.twist.angular_z = 0.0
        self.cartesian_velocity_pub.publish(velocity_vector)































