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

class PickAndPlace(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.fam = FullArmMovement()
        self.boundary_safety = rospy.get_param("~boundary_safety", None)
        self.joint_angles = rospy.get_param("~joint_angles", None)
        if self.boundary_safety is None or self.joint_angles is None:
            rospy.logerr("Joint angles or boundary_safety not defined.")
            sys.exit(1)
        self.perception_pose = None
        self.listener = tf.TransformListener()

        # how long to wait for transform (in seconds)
        self.wait_for_transform = 0.1

        self.transform_tries = 5

        # Subscribers
        self.perception_pose_sub = rospy.Subscriber('~pose_in', PoseStamped, self.perception_pose_cb)
        self.event_in_sub = rospy.Subscriber('~event_in', String, self.event_in_cb)
        self.event_out_pub = rospy.Publisher('~event_out', String, queue_size=1)
        self.debug_pose_pub = rospy.Publisher('~debug_pose', PoseStamped, queue_size=1)

        self.setup_arm_for_pick()
        rospy.sleep(3.0)
        rospy.loginfo("READY!")
        rospy.sleep(3.0)
        rospy.loginfo("READY!")

    def test_go_to_board(self):
        msg = PoseStamped()
        msg.header.frame_id = 'board_link' #board link is the name of tf
        msg.header.stamp = rospy.Time.now()
        #make the z axis (blux in rviz) face below  by rotating around x axis
        q = list(tf.transformations.quaternion_from_euler(3.14, 0.0, 0.0))
        msg.pose.orientation = Quaternion(*q)
        msg.pose.position.z += 0.2
        # Creating a zero pose of the baord link and trasnforming it with respect to base link
        msg = self.get_transformed_pose(msg, 'base_link')
        print (msg)
        debug_pose = copy.deepcopy(msg)
        self.debug_pose_pub.publish(debug_pose)
        self.fam.send_cartesian_pose(debug_pose)
        
        

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
        self.fam.example_send_gripper_command(0.5) #half close the gripper 
        #self.fam.example_send_gripper_command(0.9) #full close the gripper 
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

if __name__ == "__main__":
    rospy.init_node('pick_and_place')
    PAP = PickAndPlace()
    PAP.test_go_to_board()
    rospy.spin()

