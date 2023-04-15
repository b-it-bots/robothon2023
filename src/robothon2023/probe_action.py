#!/usr/bin/env python3

import rospy
from robothon2023.abstract_action import AbstractAction
from geometry_msgs.msg import PoseStamped, Quaternion
import geometry_msgs.msg
from sensor_msgs.msg import Image
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from utils.kinova_pose import KinovaPose, get_kinovapose_from_list, get_kinovapose_from_pose_stamped

from kortex_driver.srv import *
from kortex_driver.msg import *

import tf
import math
import cv2
import cv_bridge
import numpy as np

class ProbeAction(AbstractAction):
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:
        super().__init__(arm, transform_utils)
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', kortex_driver.msg.TwistCommand, queue_size=1)
        self.door_knob_pose_pub = rospy.Publisher("/door_knob_pose", PoseStamped, queue_size=1)
        self.probe_cable_dir_debug_pub = rospy.Publisher('/probe_cable_dir_debug', Image, queue_size=1)
        self.debug = rospy.get_param("~debug", False)
        self.bridge = cv_bridge.CvBridge()

    def pre_perceive(self) -> bool:
        print ("in pre perceive")        
        
        return True

    def act(self) -> bool:
        # success = self.place_probe_in_holder()
       
        success = self.pick_magnet()
        
        return success

    def verify(self) -> bool:
        print ("in verify")
        return True
    
    def place_probe_in_holder(self):
        '''
        Place the probe in the holder
        '''

        # go the probe initial position
        success = self.arm.execute_gripper_command(0.0) # open the gripper

        # get the probe initial position from tf
        msg = PoseStamped()
        msg.header.frame_id = "probe_initial_link"
        msg.header.stamp = rospy.Time(0)
        probe_initial_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True, offset=[0, 0, math.pi/2])

        # convert the probe initial position to a kinova pose
        probe_initial_pose_kp = get_kinovapose_from_pose_stamped(probe_initial_pose)

        # send the probe initial position to the arm
        print("[probe_action] moving to probe initial position")
        success = self.arm.send_cartesian_pose(probe_initial_pose_kp)

        if not success:
            rospy.logerr("[probe_action] Failed to move to the probe initial position")
            return False
        
        print('[probe_action] reached probe initial position')
        
        # close the gripper
        success = self.arm.execute_gripper_command(1.0)

        if not success:
            rospy.logerr("Failed to close the gripper")
            return False
        
        # remove the probe from the box
        msg = PoseStamped()
        msg.header.frame_id = "probe_initial_link"
        msg.header.stamp = rospy.Time(0)
        probe_initial_pose_in_bdl = self.transform_utils.transformed_pose_with_retries(msg, "board_link")

        probe_initial_pose_in_bdl.pose.position.x -= 0.08

        # convert the pose to base_link
        probe_mb_pose_in_bl = self.transform_utils.transformed_pose_with_retries(probe_initial_pose_in_bdl,
                                                                                 "base_link", 
                                                                                 execute_arm=True, 
                                                                                 offset=[0, 0, math.pi/2])

        # convert to a kinova pose
        move_back_pose = get_kinovapose_from_pose_stamped(probe_mb_pose_in_bl)

        print("[probe_action] moving back the probe by 5cm")
        success = self.arm.send_cartesian_pose(move_back_pose)

        if not success:
            rospy.logerr("Failed to move back the probe")
            return False
        
        print("[probe_action] moved back the probe by 5cm")

        # move up a bit
        move_up_pose = move_back_pose
        move_up_pose.x += 0.10
        move_up_pose.y -= 0.30
        move_up_pose.z += 0.25

        print("[probe_action] moving up the probe")
        success = self.arm.send_cartesian_pose(move_up_pose)

        # send joint angles to the arm
        # [0.8197946865095573, 1.7372213912136376, 1.6281319213773762, -0.7847078720428993, 0.06814585646743704, -1.4455723618104983, -3.093051482717028, 0.5578947226687478]
        
        joint_angles = [0.8197946865095573, 1.7372213912136376, 1.6281319213773762,
                        -0.7847078720428993, 0.06814585646743704, -1.4455723618104983, -3.093051482717028, 0.5578947226687478]
        
        # convert the joint angles to degrees
        joint_angles = [math.degrees(ja) for ja in joint_angles]

        # rotate joinnt 7 by 180 degrees
        joint_angles[6] += 180.0
        
        print("[probe_action] sending joint angles")
        success = self.arm.send_joint_angles(joint_angles)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        print("moved up the probe to above the holder")

        # get the current pose of the arm
        current_pose = self.arm.get_current_pose()

        # move down a bit
        current_pose.z -= 0.105
        current_pose.x += 0.005

        success = self.arm.send_cartesian_pose(current_pose)

        if not success:
            rospy.logerr("Failed to move down the probe")
            return False
        
        print("moved down the probe to above the holder")
        
        # open the gripper
        success = self.arm.execute_gripper_command(0.0)

        if not success:
            rospy.logerr("Failed to open the gripper")
            return False

        # move up a bit by 20cm
        current_pose.z += 0.2
        
        success = self.arm.send_cartesian_pose(current_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        return True
    
    def create_twist_from_velocity(self, velocity) -> Twist:
        """
        Create Twist message from velocity vector

        input: velocity vector :: np.array
        output: velocity vector :: Twist
        """
        board_wrt_base = self.transform_utils.get_pose_from_link('base_link', 'board_link')

        # convert pose.orientation to yaw
        orientation = board_wrt_base.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        print("yaw: ", yaw)

        velocity_vector = geometry_msgs.msg.Twist()
        velocity_vector.linear.x = velocity * math.cos(yaw)
        velocity_vector.linear.y = velocity * math.sin(yaw)
        velocity_vector.linear.z = 0

        velocity_vector.angular.x = 0
        velocity_vector.angular.y = 0
        velocity_vector.angular.z = 0
               
        return velocity_vector

    def pick_magnet(self):
        # # get magnet pose from param server
        # magnet_pose = rospy.get_param("~magnet_pose")
        # magnet_kinova_pose = get_kinovapose_from_list(magnet_pose)

        # # rotate the yaw by 180 degrees
        # magnet_kinova_pose.theta_z_deg += 180.0

        # # send magnet pose to the arm
        # print("[probe_action] moving to magnet position")
        # success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        # if not success:
        #     rospy.logerr("Failed to move to the magnet position")
        #     return False
        
        # print("[probe_action] reached magnet position")

        # # close the gripper
        # success = self.arm.execute_gripper_command(0.7)

        # if not success:
        #     rospy.logerr("Failed to close the gripper")
        #     return False
        
        # # move up a bit
        # magnet_kinova_pose.z += 0.3

        # success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        # if not success:
        #     rospy.logerr("Failed to move up the probe")
        #     return False
        
        # go to the door knob position
        # get the door knob position from tf
        msg = PoseStamped()
        msg.header.frame_id = "door_knob_link"
        msg.header.stamp = rospy.Time(0)
        msg.pose.position.x -= 0.015
        # msg.pose.orientation.z += 180

        door_knob_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True)

        # convert the door knob pose to a kinova pose
        door_knob_kinova_pose = get_kinovapose_from_pose_stamped(door_knob_pose)

        # move up a bit
        door_knob_kinova_pose.z += 0.022

        # send the door knob pose to the arm
        print("[probe_action] moving to door knob position")
        print("[probe_action] door knob pose: {}".format(door_knob_kinova_pose))

        success = self.arm.send_cartesian_pose(door_knob_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the door knob position")
            return False
        
        print("[probe_action] reached door knob position")

        # linear_vel_z = 0.0025
        vel = 0.05

        angle = 45.0
        # w.r.t board link
        linear_vel_x = vel*math.cos(math.radians(angle))
        linear_vel_z = vel*math.sin(math.radians(angle))

        twist_base = self.create_twist_from_velocity(linear_vel_x)
        twist_base.linear.z = linear_vel_z
        angular_vel_y = -0.1

        msg = kortex_driver.msg.TwistCommand()


        for i in range(4):

            msg.twist.linear_z = twist_base.linear.z
            msg.twist.linear_y = twist_base.linear.y 
            msg.twist.linear_x = twist_base.linear.x
            msg.twist.angular_y = angular_vel_y
            self.cart_vel_pub.publish(msg)

            twist_base.linear.z *= 0.18
            angular_vel_y -= 0.15


            rospy.sleep(1)

        msg.twist.linear_z = 0.0
        msg.twist.linear_x = 0.0
        msg.twist.linear_y = 0.0
        msg.twist.angular_y = 0.0

        self.cart_vel_pub.publish(msg)

        self.arm.clear_faults()
        self.arm.subscribe_to_a_robot_notification()
    
        # # get current pose of the arm
        # current_pose = self.arm.get_current_pose()

        # # go up by 5cm
        # current_pose.z += 0.1

        # success = self.arm.send_cartesian_pose(current_pose)

        # if not success:
        #     rospy.logerr("Failed to move up the probe")
        #     return False
        
        # # go to the magnet position
        # print("[probe_action] moving to magnet position")
        # magnet_pose = rospy.get_param("~magnet_pose")
        # magnet_kinova_pose = get_kinovapose_from_list(magnet_pose)
        # magnet_kinova_pose.theta_z_deg += 180.0
        # success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        # if not success:
        #     rospy.logerr("Failed to move to the magnet position")
        #     return False

        # # open the gripper
        # success = self.arm.execute_gripper_command(0.0)

        # if not success:
        #     rospy.logerr("Failed to open the gripper")
        #     return False
        
        # # move up a bit
        # magnet_kinova_pose.z += 0.3

        # success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        # if not success:
        #     rospy.logerr("Failed to move up the probe")
        #     return False

        # print("[probe_action] target reached")

    def push_door(self):  # push door is not cuurently used
            # move arm above door knob position 


            msg = PoseStamped()
            msg.header.frame_id = "door_knob_rotated"
            msg.header.stamp = rospy.Time(0)

            msg.pose.position.x -= 0.075
            msg.pose.position.z += 0.30

            door_knob_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True)

            print("Door knob pose:  ")
            print(door_knob_pose)
            self.door_knob_pose_pub.publish(door_knob_pose)

            rospy.sleep(5)

            success = self.arm.execute_gripper_command(1.0)

            if not success:
                rospy.logerr("Failed to open the gripper")
                return False
            

            # convert the door knob pose to a kinova pose
            door_knob_kinova_pose = get_kinovapose_from_pose_stamped(door_knob_pose)



            print("Door knob kinova pose:  ")
            print(door_knob_kinova_pose)

            success = self.arm.send_cartesian_pose(door_knob_kinova_pose)

            if not success:
                rospy.logerr("Failed to go to up doorknob position")
                return False

            print("[probe_action] door knob UP position reached")


            distance = 0.22 # m  (0.045 is the length the slider can travel and 0.005 is a small offset for safety reasons)
            time = 5 # s

            velocity = distance/time


            approach_vel_msg = kortex_driver.msg.TwistCommand()

            approach_vel_msg.twist.linear_z = -velocity
            self.cart_vel_pub.publish(approach_vel_msg)
            rospy.sleep(time)
            self.stop_arm()

            success = self.arm.execute_gripper_command(1.0)

            if not success:
                rospy.logerr("Failed to open the gripper")
                return False

            distance = 0.085     # m  (0.045 is the length the slider can travel and 0.005 is a small offset for safety reasons)
            time = 6 # s

            velocity = distance/time

            msg = self.create_velocity(velocity,door_knob_pose)
            self.cart_vel_pub.publish(msg)
            rospy.sleep(time)
            self.stop_arm()

            self.arm.clear_faults()
            self.arm.subscribe_to_a_robot_notification()

            print("[probe_action] door pushed and process finished")

            return True
        

    def create_velocity(self,velocity, pose):
        """
        Create Twist message from velocity vector

        input: velocity vector :: np.array
        output: velocity vector :: Twist
        """


        # convert pose.orientation to yaw
        orientation = pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        print("yaw: ", yaw)

        msg = kortex_driver.msg.TwistCommand()

        msg.twist.linear_x = velocity * math.cos(yaw)
        msg.twist.linear_y = velocity * math.sin(yaw)
        msg.twist.linear_z = 0.0
        msg.duration = 20
        return msg
    

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
        self.cart_vel_pub.publish(velocity_vector)

        return True


    
    def pick_probe(self):
        
        # go to the probe pre-pick position above the holder
        probe_holder_pick_pre_pose = rospy.get_param("~probe_holder_pick_pre_pose")
        probe_holder_pick_pre_kinova_pose = get_kinovapose_from_list(probe_holder_pick_pre_pose)
        
        print("[probe_action] moving to probe holder pick pre position")
        success = self.arm.send_cartesian_pose(probe_holder_pick_pre_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the probe holder pick pre position")
            return False
        
        print("[probe_action] reached probe holder pick pre position")

        # get an image from the camera
        image_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # get the probe cable direction
        probe_cable_dir = self.get_probe_cable_dir(image)

        if probe_cable_dir > 0:
            probe_cable_dir += 90
        elif probe_cable_dir < 0:
            probe_cable_dir = -probe_cable_dir
        
        # get the probe pick from holder pose
        probe_pick_from_holder_pose = rospy.get_param("~probe_pick_from_holder_pose")
        probe_pick_from_holder_kinova_pose = get_kinovapose_from_list(probe_pick_from_holder_pose)

        pre_pick_pose = probe_pick_from_holder_kinova_pose
        pre_pick_pose.z = probe_holder_pick_pre_kinova_pose.z

        # move to the pre pick pose
        print("[probe_action] moving to probe pre pick position")
        success = self.arm.send_cartesian_pose(pre_pick_pose)

        if not success:
            rospy.logerr("Failed to move to the probe pre pick position")
            return False
        
        print("[probe_action] reached probe pre pick position")

        probe_pick_from_holder_pose = rospy.get_param("~probe_pick_from_holder_pose")
        probe_pick_from_holder_kinova_pose = get_kinovapose_from_list(probe_pick_from_holder_pose)
        # rotate the arm to the probe cable direction
        probe_pick_from_holder_kinova_pose.theta_z_deg = probe_cable_dir

        # move to the probe pick from holder pose
        print("[probe_action] moving to probe pick from holder position")
        success = self.arm.send_cartesian_pose(probe_pick_from_holder_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the probe pick from holder position")
            return False
        
        print("[probe_action] reached probe pick from holder position")

        # close the gripper
        success = self.arm.execute_gripper_command(1.0)

        if not success:
            rospy.logerr("Failed to close the gripper")
            return False
        
        # move up a bit
        probe_pick_from_holder_kinova_pose.z += 0.15

        success = self.arm.send_cartesian_pose(probe_pick_from_holder_kinova_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        print("[probe_action] probe picked")

        return True


    def get_probe_cable_dir(self, image):
        '''
        This function takes an image of the probe cable and returns the direction of the cable
        return: angle in degrees (in cv coordinate system)
        '''

        # crop the border of the image by 25%
        image = image[int(image.shape[0]*0.25):int(image.shape[0]*0.75), int(image.shape[1]*0.25):int(image.shape[1]*0.75)]

        # Convert image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # find the orange object in the image 
        lower_orange = np.array([0, 100, 100])
        upper_orange = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # draw the contours of the orange object in the image
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

        # filter the small contours
        contours = [c for c in contours if cv2.contourArea(c) > 100]

        # plot the center of the orange object in the image
        M = cv2.moments(contours[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # find the direction of the black cable from the center of the orange object
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 30])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > 100]
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

        M = cv2.moments(contours[0])
        bX = int(M["m10"] / M["m00"])
        bY = int(M["m01"] / M["m00"])

        # plot the direction of the cable with an arrow
        cv2.arrowedLine(image, (cX, cY), (bX, bY), (255, 0, 0), 2)

        # print the angle of the cable direction wrt center
        angle = math.atan2(bY - cY, bX - cX) * 180.0 / math.pi

        # plot the angle of the cable direction wrt center
        cv2.putText(image, "angle: {:.2f} degrees".format(angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # publish the debug image to the topic
        if self.debug:
            self.probe_cable_dir_debug_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

        return angle
