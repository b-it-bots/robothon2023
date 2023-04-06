
#!/usr/bin/env python3
#
import rospy
import tf
from robothon2023.abstract_action import AbstractAction
from robothon2023.transform_utils import TransformUtils
from geometry_msgs.msg import PoseStamped, Quaternion
import kortex_driver.msg

class ButtonPressAction(AbstractAction):
    """
    Assumption: From top view the approximate pose of the 
    buttons is available 
    
    - Move the arm to the top of the buttons
    - Do visual servoving and adjust arm 
    - Move arm with velocity control
    - stop when the force is higher 
    - retreat above 
    """
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:
        super(ButtonPressAction, self).__init__(arm, transform_utils)
        self.base_feedback_sub = rospy.Subscriber('/my_gen3/base_feedback', kortex_driver.msg.BaseCyclic_Feedback, self.base_feedback_cb)
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', kortex_driver.msg.TwistCommand, queue_size=1)
        self.current_force_z = []

    def base_feedback_cb(self, msg):
        self.current_force_z.append(msg.base.tool_external_wrench_force_z)
        if len(self.current_force_z) > 25:
            self.current_force_z.pop(0)

    def pre_perceive(self) -> bool:
        print ("in pre perceive")

        pre_height_above_button = rospy.get_param("~pre_height_above_button", 0.1)
        ## Getting Pose of the board_link in the base_link 
        msg = PoseStamped()
        msg.header.frame_id = 'board_link' #board link is the name of tf
        msg.header.stamp = rospy.Time.now()
        #make the z axis (blux in rviz) face below  by rotating around x axis
        q = list(tf.transformations.quaternion_from_euler(math.pi, 0.0, math.pi/2))
        msg.pose.orientation = Quaternion(*q)
        msg.pose.position.z += pre_height_above_button
        # either because of a camera calibration offset or something to do with the reference frame for sending Cartesian poses
#        msg.pose.position.x += 0.01
        # Creating a zero pose of the baord link and trasnforming it with respect to base link
        msg = self.transform_utils.transformed_pose_with_retries(msg, 'base_link')
        #debug_pose = copy.deepcopy(msg)
        #self.debug_pose_pub.publish(debug_pose)
        self.arm.send_cartesian_pose(debug_pose)
        return True

    def act(self) -> bool:
        print ("in act")
        linear_vel_z = rospy.get_param("~linear_vel_z", 0.005)
        force_z_diff_threshold = rospy.get_param("~force_z_diff_threshold", 3.0)
        stop = False
        self.current_force_z = []
        num_retries = 0
        while not rospy.is_shutdown():
            if len(self.current_force_z) < 20:
                num_retries += 1
                if num_retries > 100:
                    rospy.logerr("No force measurements received")
                    break
                self.loop_rate.sleep()
                continue
            msg = kortex_driver.msg.TwistCommand()
            msg.twist.linear_z = -linear_vel_z
            if abs(np.mean(self.current_force_z) - self.current_force_z[-1]) > force_z_diff_threshold:
                stop = True
                msg.twist.linear_z = 0.0
            self.cart_vel_pub.publish(msg)
            if stop:
                break
            self.loop_rate.sleep()
        msg = kortex_driver.msg.TwistCommand()
        msg.twist.linear_z = linear_vel_z
        for idx in range(5):
            self.cart_vel_pub.publish(msg)
            self.loop_rate.sleep()
        msg.twist.linear_z = 0.0
        self.cart_vel_pub.publish(msg)
        self.loop_rate.sleep()
        return True

    def verify(self) -> bool:
        print ("in verify")
        return True
