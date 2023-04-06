
#!/usr/bin/env python3
#
import tf
import rospy
import numpy as np
from robothon2023.abstract_action import AbstractAction
from robothon2023.full_arm_movement import FullArmMovement
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3
from robothon2023.transform_utils import TransformUtils

from kortex_driver.msg import TwistCommand

class SliderAction(AbstractAction):

    def __init__(self, arm: FullArmMovement):
        super().__init__(arm)
        self.arm = arm
    
        self.tf_utils = TransformUtils()
        self.listener = tf.TransformListener()
        self.slider_pose = PoseStamped()

        self.slider_pose_sub = rospy.Subscriber('/task_board_detector/slider_start_pose', PoseStamped, self.slider_pose_callback, queue_size=1)
        self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
        
        print("Slider Action Initialized")
        

    def pre_perceive(self) -> bool:
        print ("in pre perceive")
        return True

    def act(self) -> bool:
        print ("in act")

        self.move_arm_to_slider()

        velocity_value = 0.05
        self.slider_velocity_twist = self.create_twist_from_velocity(velocity_value)
        
        self.move_arm_along_slider()
        rospy.sleep(3.0)
        self.stop_arm()

        return True

    def verify(self) -> bool:
        print ("in verify")
        return True

    def do(self) -> bool:
        return self.act()

    
    def slider_pose_callback(self, msg):
        """
        Callback function for slider pose
        """
        self.slider_pose.pose.position.x = msg.pose.position.x
        self.slider_pose.pose.position.y = msg.pose.position.y
        self.slider_pose.pose.position.z = msg.pose.position.z
        self.slider_pose.pose.orientation.x = msg.pose.orientation.x
        self.slider_pose.pose.orientation.y = msg.pose.orientation.y
        self.slider_pose.pose.orientation.z = msg.pose.orientation.z
        self.slider_pose.pose.orientation.w = msg.pose.orientation.w


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


    def move_arm_to_slider(self):
        """
        Move arm to along slider in with velocity vector
        """

        rospy.loginfo(" ===== >> Moving arm to slider << ======")
        print(self.slider_pose)
        self.arm.send_cartesian_pose(self.slider_pose)
        rospy.sleep(1.0)
        rospy.loginfo(" ===== >> stop << ======")


    def move_arm_along_slider(self):
        """
        Move arm along slider in with velocity vector
        """
        print(self.slider_velocity_twist)
        slider_velocity = self.convert_twist_twistcommand(self.slider_velocity_twist)
        self.cartesian_velocity_pub.publish(slider_velocity)


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

































## CODE FLOW ##

    # 1. SliderAction is initialized with the arm object

    # 2. SliderAction subscribes to slider_pose topic

    # 3. SliderAction creates slider_direction vector

    # 4. SliderAction converts slider_direction vector to base frame

    # 5. SliderAction moves arm to slider

    # 6. SliderAction sends slider_direction vector to arm

    # 7. SliderAction moves arm along slider

    # 8. SliderAction stops arm