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
        # pick the probe from the box and place it in the holder
        success = self.pluck_place_probe_in_holder()

        if not success:
            rospy.logerr("[probe_action] Failed to place the probe in the holder")
            return False
        
        # # open the door with magnet
        # success = self.open_door()

        # if not success:
        #     rospy.logerr("[probe_action] Failed to open the door")
        #     return False
        
        # # pick the probe from the holder
        # success = self.pick_probe_from_holder()
        
        # # probe the circuit
        # success = self.probe_circuit()

        # if not success:
        #     rospy.logerr("[probe_action] Failed to probe the circuit")
        #     return False
        
        # # place the probe somewhere safe
        # success = self.place_probe_safe()

        # if not success:
        #     rospy.logerr("[probe_action] Failed to place the probe somewhere safe")
        #     return False
        
        return success

    def verify(self) -> bool:
        print ("in verify")
        return True
    
    def pluck_place_probe_in_holder(self):
        '''
        Pluck the cable probe from the box and place it in the holder
        '''

        # go the probe initial position
        success = self.arm.execute_gripper_command(0.5) # open the gripper

        # pluck the probe from the box
        success = self.pluck_probe_from_box()

        if not success:
            rospy.logerr("[probe_action] Failed to pluck the probe from the box")
            return False
        
        # place the probe in the holder
        # success = self.place_probe_in_holder()

        # if not success:
        #     rospy.logerr("[probe_action] Failed to place the probe in the holder")
        #     return False
        
        return True
        
    
    def pluck_probe_from_box(self):
        # get the probe initial position from tf
        msg = PoseStamped()
        msg.header.frame_id = "probe_initial_link"
        msg.header.stamp = rospy.Time(0)
        probe_initial_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True, offset=[0, 0, math.pi/2])

        # convert the probe initial position to a kinova pose
        probe_initial_pose_kp = get_kinovapose_from_pose_stamped(probe_initial_pose)

        # increase the z position by 5cm
        probe_initial_pose_kp.z += 0.1

        # send the probe initial position to the arm
        print("[probe_action] moving to probe initial position")
        success = self.arm.send_cartesian_pose(probe_initial_pose_kp)

        if not success:
            rospy.logerr("[probe_action] Failed to move to the probe initial position")
            return False
        
        print('[probe_action] reached probe initial position')

        # use velocity control to move the probe down
        print("[probe_action] moving down the probe")
        success = self.arm.move_down_with_caution(force_threshold=[2,2,2])

        if not success:
            rospy.logerr("[probe_action] Failed to move down the probe")
            return False
        
        print('[probe_action] moved down the probe')
        
        # close the gripper
        success = self.arm.execute_gripper_command(0.8)

        if not success:
            rospy.logerr("Failed to close the gripper")
            return False

        # move the probe back in x direction for 6cm
        success = self.arm.move_with_velocity(-0.06, 3, 'y')

        if not success:
            rospy.logerr("Failed to move back the probe")
            return False

        probe_current_pose = self.arm.get_current_pose()

        probe_current_pose.z += 0.25

        print("[probe_action] moving up the probe")
        success = self.arm.send_cartesian_pose(probe_current_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        print("[probe_action] moved up the probe")

        return True

    def place_probe_in_holder(self):
        # move the probe to the holder
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

    def open_door(self):
        # get magnet pose from param server
        magnet_pose = rospy.get_param("~magnet_pose")
        magnet_kinova_pose = get_kinovapose_from_list(magnet_pose)

        # rotate the yaw by 180 degrees
        magnet_kinova_pose.theta_z_deg += 180.0

        # send magnet pose to the arm
        print("[probe_action] moving to magnet position")
        success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the magnet position")
            return False
        
        print("[probe_action] reached magnet position")

        # close the gripper
        success = self.arm.execute_gripper_command(0.7)

        if not success:
            rospy.logerr("Failed to close the gripper")
            return False
        
        # move up a bit
        magnet_kinova_pose.z += 0.3

        success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False
        
        # go to the door knob position
        # get the door knob position from tf
        msg = PoseStamped()
        msg.header.frame_id = "door_knob_link"
        msg.header.stamp = rospy.Time(0)
        msg.pose.position.x -= 0.015

        door_knob_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True, offset=[0.0, 0.0, -90.0])

        # convert the door knob pose to a kinova pose
        door_knob_kinova_pose = get_kinovapose_from_pose_stamped(door_knob_pose)

        # move up a bit
        door_knob_kinova_pose.z += 0.022 + 0.03

        # send the door knob pose to the arm
        print("[probe_action] moving to door knob position")
        print("[probe_action] door knob pose: {}".format(door_knob_kinova_pose))

        success = self.arm.send_cartesian_pose(door_knob_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the door knob position")
            return False
        
        print("[probe_action] reached door knob position")

        # linear_vel_z = 0.0025
        vel = 0.03

        angle = 25.0
        # w.r.t board link
        linear_vel_x = vel*math.cos(math.radians(angle))
        linear_vel_z = vel*math.sin(math.radians(angle))

        twist_base = self.create_twist_from_velocity(linear_vel_x)
        twist_base.linear.z = linear_vel_z
        angular_vel_x = -0.02

        msg = kortex_driver.msg.TwistCommand()
        # twist_base.linear.z *= 0.5

        print("twist_base.linear.z: ", twist_base.linear.z)
        for i in range(3):

            msg.twist.linear_z = twist_base.linear.z
            msg.twist.linear_y = twist_base.linear.y 
            msg.twist.linear_x = twist_base.linear.x
            msg.twist.angular_x = -abs(angular_vel_x)
            self.cart_vel_pub.publish(msg)

            # reduce velocity in z direction 

            twist_base.linear.z = abs(math.log(twist_base.linear.z+1.0))
            print("twist_base.linear.z: ", twist_base.linear.z)
            print("angular_vel_x: ", angular_vel_x)



            angular_vel_x -= 0.05
            rospy.sleep(1)

        msg.twist.linear_z = 0.0
        msg.twist.linear_x = 0.0
        msg.twist.linear_y = 0.0
        msg.twist.angular_x = 0.0
        msg.twist.angular_y = 0.0
        msg.twist.angular_z = 0.0

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
        msg.header.frame_id = "door_knob_link"
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
            rospy.logerr("Failed to move up the probe")
            return False
        
        # go to the magnet position
        print("[probe_action] moving to magnet position")
        magnet_pose = rospy.get_param("~magnet_pose")
        magnet_kinova_pose = get_kinovapose_from_list(magnet_pose)
        magnet_kinova_pose.theta_z_deg += 180.0
        success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        if not success:
            rospy.logerr("Failed to move to the magnet position")
            return False

        # open the gripper
        success = self.arm.execute_gripper_command(0.0)

        if not success:
            rospy.logerr("Failed to open the gripper")
            return False
        
        # move up a bit
        magnet_kinova_pose.z += 0.3

        success = self.arm.send_cartesian_pose(magnet_kinova_pose)

        if not success:
            rospy.logerr("Failed to move up the probe")
            return False

        print("[probe_action] target reached")    
    
    def pick_probe_from_holder(self):
        
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

    def probe_circuit(self):
        pass

    def place_probe_safe(self):
        pass

    def get_probe_cable_dir(self, image):
        '''
        This function takes an image of the probe cable and returns the direction of the cable
        return: angle in degrees (in cv coordinate system)
        '''

        # crop the border of the image by 25%
        image = image[int(image.shape[0]*0.25):int(image.shape[0]*0.75), int(image.shape[1]*0.25):int(image.shape[1]*0.75)]

        # Convert image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # find the gray color holder object in the image 
        lower_gray = np.array([0, 0, 100])
        upper_gray = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_gray, upper_gray)

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
