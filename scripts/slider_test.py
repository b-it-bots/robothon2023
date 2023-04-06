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

class RobothonTask(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.fam = FullArmMovement()
        self.slider_action = SliderAction(self.fam)

        self.perception_pose = None
        self.listener = tf.TransformListener()

        # how long to wait for transform (in seconds)
        self.wait_for_transform = 0.1

        self.transform_tries = 5


        self.base_feedback_sub = rospy.Subscriber('/my_gen3/base_feedback', kortex_driver.msg.BaseCyclic_Feedback, self.base_feedback_cb)
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', kortex_driver.msg.TwistCommand, queue_size=1)
        self.loop_rate = rospy.Rate(10.0)
        self.current_force_z = []

        #self.setup_arm_for_pick()
        #rospy.sleep(3.0)
        rospy.loginfo("READY!")
        #rospy.sleep(3.0)
        rospy.loginfo("READY!")
    
    def traverse_waypoints(self, waypoints):
        '''
        waypoints: list of waypoints to traverse.\n
        each waypoint is a list of 6 floats: [x, y, z, roll, pitch, yaw].\n
        angles are in degrees.
        '''

        # move the arm through the waypoints
        feedback = rospy.wait_for_message("/" + self.fam.robot_name + "/base_feedback", BaseCyclic_Feedback)

        client = actionlib.SimpleActionClient('/' + self.fam.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', 
                                              kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        # create waypoints
        for waypoint in waypoints:
            goal.trajectory.append(self.FillCartesianWaypoint(waypoint[0], waypoint[1], waypoint[2],
                                                                math.radians(waypoint[3]), math.radians(waypoint[4]), math.radians(waypoint[5]), 0.0))
            
        
        goal.use_optimal_blending = True

        # Call the service
        rospy.loginfo("Sending goal(Cartesian waypoint) to action server...")
        try:
            client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            client.wait_for_result()
            return True

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
       
        return cartesianWaypoint

    def generate_point_to_point_waypoints(self, target_pose: PoseStamped):
        '''
        input: target pose in base frame\n

        generate waypoints for point to point motion to avoid collisions
        '''

        # generate waypoints for point to point motion
        waypoints = []
        
        feedback = rospy.wait_for_message("/" + self.fam.robot_name + "/base_feedback", BaseCyclic_Feedback)

        # convert the target pose quaternion to euler angles
        target_pose_euler = tf.transformations.euler_from_quaternion(
            [
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w
            ]
        )

        waypoints.append(
            feedback.base.commanded_tool_pose_x,
            feedback.base.commanded_tool_pose_y,
            feedback.base.commanded_tool_pose_z + 0.05,
            feedback.base.commanded_tool_pose_theta_x,
            feedback.base.commanded_tool_pose_theta_y,
            feedback.base.commanded_tool_pose_theta_z
        )

        waypoints.append(
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z + 0.05,
            target_pose_euler[0],
            target_pose_euler[1],
            target_pose_euler[2]
        )

        waypoints.append(
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
            target_pose_euler[0],
            target_pose_euler[1],
            target_pose_euler[2]
        )

        return waypoints
    
    def get_pose_from_link(self, link_name: str):
        '''
        input: link_name\n
        output: PoseStamped\n
        returns the pose of the link in the base_link frame
        '''

        msg = PoseStamped()
        msg.header.frame_id = link_name
        msg.header.stamp = rospy.Time.now()
        msg = self.get_transformed_pose(msg, 'base_link')

        return msg

    def base_feedback_cb(self, msg):
        self.current_force_z.append(msg.base.tool_external_wrench_force_z)
        if len(self.current_force_z) > 25:
            self.current_force_z.pop(0)

    def test_go_to_board(self):
        pre_height_above_button = rospy.get_param("~pre_height_above_button", 0.1)
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
        msg = self.get_transformed_pose(msg, 'base_link')
        print (msg)
        debug_pose = copy.deepcopy(msg)
        self.debug_pose_pub.publish(debug_pose)
        self.fam.send_cartesian_pose(debug_pose)

    def test_press_button(self):
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


    def perception_pose_cb(self, msg):
        msg = self.get_transformed_pose(msg, 'base_footprint')
        rospy.loginfo(msg.pose)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        print ("Extract yaw ", yaw)
        print ("Extract roll ", roll)
        print ("Extract pitch ", pitch)
        
        #q = list(tf.transformations.quaternion_from_euler(0, 0, yaw))
        #Lot of assumptions here 
        # Tried sending arm to different poses and figured this 
        q = list(tf.transformations.quaternion_from_euler(3.14, 0.0, yaw))
        msg.pose.orientation = Quaternion(*q)

        #rospy.loginfo(msg.pose)
        print(self.boundary_safety)
        print("x safety check pass " , self.boundary_safety["x_min"] < msg.pose.position.x < self.boundary_safety["x_max"])
        print("y safety check pass ", self.boundary_safety["y_min"] < msg.pose.position.y < self.boundary_safety["y_max"])
        print("z safety check pass ", self.boundary_safety["z_min"] < msg.pose.position.z < self.boundary_safety["z_max"])
        if self.boundary_safety["x_min"] < msg.pose.position.x < self.boundary_safety["x_max"] and \
                self.boundary_safety["y_min"] < msg.pose.position.y < self.boundary_safety["y_max"] and \
                self.boundary_safety["z_min"] < msg.pose.position.z < self.boundary_safety["z_max"]:

            ##Raising the perception pose in Z in basefootprint 
            msg.pose.position.z = msg.pose.position.z + 0.15

            #Saving it back in base link
            msg = self.get_transformed_pose(msg, 'base_link')
            self.perception_pose = msg


            #Copied code from event_in_cb
            #publishes debug pose for visualziation
            debug_pose = copy.deepcopy(self.perception_pose)
            self.debug_pose_pub.publish(debug_pose)
        else:
            rospy.logerr("Input pose out of bound")



    def event_in_cb(self, msg):
        print ("Received message ", msg.data)
        if msg.data == 'e_demo':
            #self.fam.test_send_joint_angles(self.joint_angles["perceive_right_pose"])
            self.fam.test_send_joint_angles(self.joint_angles["demo_pose"])
            rospy.sleep(2)

        if msg.data == 'e_perceive_right':
            self.fam.test_send_joint_angles(self.joint_angles["perceive_right_pose"])
        
        if msg.data == 'e_perceive_left':
            self.fam.test_send_joint_angles(self.joint_angles["perceive_left_pose"])

        if msg.data == 'e_pick_right' or msg.data == 'e_pick_left':
            if self.perception_pose is None:
                #Send back feedback 
                self.event_out_pub.publish('e_done')
                return
            debug_pose = copy.deepcopy(self.perception_pose)
            #debug_pose.pose.position.z = self.fam.current_ee_pose[2] - 0.03
            #theta_x, theta_y, theta_z = tf.transformations.euler_from_quaternion((
            #        debug_pose.pose.orientation.x, debug_pose.pose.orientation.y,
            #        debug_pose.pose.orientation.z, debug_pose.pose.orientation.w))
            # debug_pose.pose.orientation = FullArmMovement.quat_from_rpy(*map(math.radians, self.fam.current_ee_pose[3:]))
            #debug_pose.pose.orientation = FullArmMovement.quat_from_rpy(
            #        math.radians(self.fam.current_ee_pose[3]),
            #        math.radians(self.fam.current_ee_pose[4]),
            #        theta_z)
            # debug_pose = self.fam.get_pose_from_current_ee_pose()
            # debug_pose.header.frame_id = 'base_link'
            #debug_pose.pose.position.z = self.perception_pose.pose.position.z + 0.1
            self.debug_pose_pub.publish(debug_pose)
            self.fam.send_cartesian_pose(debug_pose)
            self.perception_pose = None
            '''
            self.fam.close_gripper()
            if msg.data == 'e_pick_right':
                self.fam.test_send_joint_angles(self.joint_angles["perceive_right_pose"])
                self.fam.test_send_joint_angles(self.joint_angles["intermediate_place_pose_left"])
                self.fam.test_send_joint_angles(self.joint_angles["place_pose_left"])
                self.fam.open_gripper()
                self.perception_pose = None
            elif msg.data == 'e_pick_left':
                self.fam.test_send_joint_angles(self.joint_angles["perceive_left_pose"])
                self.fam.test_send_joint_angles(self.joint_angles["intermediate_place_pose_right"])
                self.fam.test_send_joint_angles(self.joint_angles["place_pose_right"])
                self.fam.open_gripper()
                self.perception_pose = None
            '''
        if msg.data == 'e_stop':
            self.perception_pose = None

        #Send back feedback 
        self.event_out_pub.publish('e_done')


    def setup_arm_for_pick(self):
        """Setup the arm to go to pick pose
        :returns: None

        """
        self.fam.example_clear_faults()
        self.fam.example_subscribe_to_a_robot_notification()
        # self.fam.test_send_joint_angles(self.joint_angles["vertical_pose"])
        print (self.joint_angles['perceive_table'])
        self.fam.example_send_joint_angles(self.joint_angles["perceive_table"])
        self.fam.example_send_gripper_command(0.0) #Open the gripper 
        #self.fam.example_send_gripper_command(0.5) #half close the gripper 
        self.fam.example_send_gripper_command(0.9) #full close the gripper 
        rospy.sleep(2.0)

    def get_transformed_pose(self, reference_pose, target_frame):
        """ Transform pose with multiple retries

        :return: The updated state.
        :rtype: str

        """
        for i in range(0, self.transform_tries):
            transformed_pose = self.transform_pose(reference_pose, target_frame)
            if transformed_pose:
                return transformed_pose
        transformed_pose = None
        return transformed_pose


    def transform_pose(self, reference_pose, target_frame):
        """
        Transforms a given pose into the target frame.

        :param reference_pose: The reference pose.
        :type reference_pose: geometry_msgs.msg.PoseStamped

        :param target_frame: The name of the taget frame.
        :type target_frame: String

        :return: The pose in the target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            common_time = self.listener.getLatestCommonTime(
                target_frame, reference_pose.header.frame_id
            )

            self.listener.waitForTransform(
                target_frame, reference_pose.header.frame_id,
                common_time, rospy.Duration(self.wait_for_transform)
            )
            reference_pose.header.stamp = common_time

            transformed_pose = self.listener.transformPose(
                target_frame, reference_pose,
            )

            return transformed_pose

        except tf.Exception as error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None
    def test(self):
        self.slider_action.do()

if __name__ == "__main__":
    rospy.init_node('robothon_task')
    task = RobothonTask()
    task.test()
    #PAP.test_go_to_board()
    #PAP.test_press_button()
    rospy.spin()
