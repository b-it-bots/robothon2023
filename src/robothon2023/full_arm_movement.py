#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
import math
import tf

from kortex_driver.srv import *
from kortex_driver.msg import *

import actionlib
from geometry_msgs.msg import PoseStamped
from utils.kinova_pose import KinovaPose

from typing import List

class FullArmMovement:
    def __init__(self):

        self.HOME_ACTION_IDENTIFIER = 2

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

        rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

        # Init the action topic subscriber
        self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
        self.last_action_notif_type = None

        # Init the services
        clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
        rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
        self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
    
        get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
        rospy.wait_for_service(get_product_configuration_full_name)
        self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

        validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
        rospy.wait_for_service(validate_waypoint_list_full_name)
        self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)

        apply_emergency_stop = '/' + self.robot_name + '/base/apply_emergency_stop'
        rospy.wait_for_service(apply_emergency_stop)
        self.apply_E_STOP = rospy.ServiceProxy(apply_emergency_stop, ApplyEmergencyStop)

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
    
    def traverse_waypoints(self, waypoints: List[KinovaPose]):
        '''
        waypoints: list of KinovaPose's to traverse.\n
        each waypoint is a list of 6 floats: [x, y, z, roll, pitch, yaw].\n
        angles are in degrees.
        '''
        self.last_action_notif_type = None
        # move the arm through the waypoints

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        # client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', 
        #                                       kortex_driver.msg.FollowCartesianTrajectoryAction)

        # rospy.loginfo("Waiting for cartesian waypoint server...")
        # client.wait_for_server()

        # goal = FollowCartesianTrajectoryGoal()

        # create waypoints
        for kinova_pose in waypoints:
             trajectory.waypoints.append(self.FillCartesianWaypoint(kinova_pose.x,
                                                                    kinova_pose.y,
                                                                    kinova_pose.z,
                                                                    kinova_pose.theta_x_deg,
                                                                    kinova_pose.theta_y_deg,
                                                                    kinova_pose.theta_z_deg,
                                                                    0.0))
        
        trajectory.use_optimal_blending = True

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Call the service
        rospy.loginfo("Sending goal(Cartesian waypoint) to action server...")
        try:
            # client.send_goal(goal)
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            # client.wait_for_result()
            # return True
            return self.wait_for_action_end_or_abort()
        
    def FillCartesianWaypointTW(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        '''
        input: x, y, z, theta_x, theta_y, theta_z, blending_radius\n
        input angles are in radians\n
        Fill CartesianWaypoint with the given parameters for actionlib waypoints method
        '''
        self.last_action_notif_type = None

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
    
    def generate_point_to_point_waypoints(self, target_pose: KinovaPose):
        '''
        input: target pose in base frame\n

        generate waypoints for point to point motion to avoid collisions
        '''

        # generate waypoints for point to point motion
        waypoints = []
        
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        waypoints.append(
            feedback.base.commanded_tool_pose_x,
            feedback.base.commanded_tool_pose_y,
            feedback.base.commanded_tool_pose_z + 0.05,
            feedback.base.commanded_tool_pose_theta_x,
            feedback.base.commanded_tool_pose_theta_y,
            feedback.base.commanded_tool_pose_theta_z
        )

        waypoints.append(
            target_pose.x,
            target_pose.y,
            target_pose.z + 0.05,
            target_pose.theta_x_deg,
            target_pose.theta_y_deg,
            target_pose.theta_z_deg
        )

        waypoints.append(
            target_pose.x,
            target_pose.y,
            target_pose.z,
            target_pose.theta_x_deg,
            target_pose.theta_y_deg,
            target_pose.theta_z_deg
        )

        return waypoints
    
    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        '''
        input: x, y, z, theta_x, theta_y, theta_z, blending_radius\n
        input angles are in degrees\n
        '''
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint
    
    def get_current_pose(self) -> KinovaPose:
        '''
        Get the current position of the robot
        
        output: current position of the robot in KinovaPose
        '''
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        current_pose = KinovaPose(0, 0, 0, 0, 0, 0)
        current_pose.x = feedback.base.commanded_tool_pose_x
        current_pose.y = feedback.base.commanded_tool_pose_y
        current_pose.z = feedback.base.commanded_tool_pose_z
        current_pose.theta_x_deg = feedback.base.commanded_tool_pose_theta_x
        current_pose.theta_y_deg = feedback.base.commanded_tool_pose_theta_y
        current_pose.theta_z_deg = feedback.base.commanded_tool_pose_theta_z

        return current_pose

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True
        
    def apply_E_stop(self):
        try:
            self.apply_E_STOP()
        except rospy.ServiceException:
            rospy.logerr("Failed to call E-stop")
            return False
        else:
            rospy.loginfo("ROBOT Stopped successfully")
            rospy.sleep(2.5)
            return True

    def home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def send_joint_angles(self, joint_angles):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        for i in range(self.degrees_of_freedom):
            angularWaypoint.angles.append(joint_angles[i])

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        print(trajectory.waypoints)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION) :
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        # Send the angles
        rospy.loginfo("Sending the robot to joint angles...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def execute_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def send_cartesian_pose(self, pose: KinovaPose):
        '''
        input: pose (PoseStamped)
        output: success (bool)
        takes in a pose and moves the arm to that pose in cartesian space
        '''
        self.last_action_notif_type = None
        
        return self.traverse_waypoints([pose])

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Activate the action notifications
            success &= self.subscribe_to_a_robot_notification()
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            success &= self.home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Example of gripper command
            # Let's fully open the gripper
            if self.is_gripper_present:
                success &= self.execute_gripper_command(0.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")  
            #*******************************************************************************

            ##*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.set_cartesian_reference_frame()

            # Example of cartesian pose
            # Let's make it move in Z
            success &= self.send_cartesian_pose()
            #*******************************************************************************

            #*******************************************************************************
            # Example of angular position
            # Let's send the arm to vertical position
            #success &= self.example_send_joint_angles()
            #*******************************************************************************

            #*******************************************************************************
            # Example of gripper command
            # Let's close the gripper at 50%
            if self.is_gripper_present:
                success &= self.execute_gripper_command(0.5)
            else:
                rospy.logwarn("No gripper is present on the arm.")    
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            success &= self.home_the_robot()
            #*******************************************************************************

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

        print ("Completed !!")


if __name__ == "__main__":
    pass
    #ex = FullArmMovement()
    #ex.main()
