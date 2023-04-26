#!/usr/bin/env python3

import rospy
from robothon2023.abstract_action import AbstractAction
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from utils.kinova_pose import get_kinovapose_from_list, get_kinovapose_from_pose_stamped
from geometry_msgs.msg import PoseStamped, Quaternion
from kortex_driver.srv import *
from kortex_driver.msg import *
import tf
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import datetime
import pdb

class WindCableAction(AbstractAction):
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:
        super().__init__(arm, transform_utils)
        self.debug = rospy.get_param("~debug", False)
        self.img_sub = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_cb)
        self.img_pub = rospy.Publisher(
            '/visual_servoing_debug_img', Image, queue_size=10)
        self.img_pub_debug_original = rospy.Publisher(
            '/visual_servoing_debug_img_original', Image, queue_size=10)
        self.img_pub_debug_contours = rospy.Publisher(
            '/visual_servoing_debug_img_contours', Image, queue_size=10)
        self.image = None
        self.loop_rate = rospy.Rate(10)
        self.bridge = CvBridge()
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', kortex_driver.msg.TwistCommand, queue_size=1)

    def pre_perceive(self) -> bool:
        print ("in pre perceive")        

        # pre-perceive pose

        # get the pre-perceive pose from tf
        msg = PoseStamped()
        msg.header.frame_id = "wind_cable_link"
        msg.header.stamp = rospy.Time(0)
        wind_cable_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True, offset=[0, 0, math.pi/2])

        # convert to kinova pose
        kp = get_kinovapose_from_pose_stamped(wind_cable_pose)

        kp.z -= 0.01

        # send to arm
        rospy.loginfo("Sending pre-perceive pose to arm")
        success = self.arm.send_cartesian_pose(kp)
        
        return success

    def act(self) -> bool:
        
        # start visual servoing
        rospy.loginfo("Starting visual servoing")
        # TODO: fix the visual servoing
        # success = self.run_visual_servoing(self.detect_wind_cable, True)

        # if not success:
        #     return False
        
        # wind cable
        success = self.wind_cable()
        
        if not success:
            return False
        
        # wind cable
        success = self.pick_probe_from_holder()
        
        if not success:
            return False

        # tuck the probe into board
        rospy.loginfo("Tucking probe into board")
        success = self.tuck_probe_into_board()

        if not success:
            return False

        return success

    def verify(self) -> bool:
        print ("in verify")
        return True
    
    def image_cb(self, msg):

        # get the image from the message
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image = image
    
    def wind_cable(self) -> bool:

        pose_num = 1
        self.arm.execute_gripper_command(0.0)
        success = False
        for i in range(1, 5):
            gripper_angle = rospy.get_param("~wind_poses/traj" + str(i)+"/gripper")
            
            poses = rospy.get_param("~wind_poses/traj" + str(i)+"/poses")
            
            # from poses dict, get number of poses
            num_poses = len(poses)

            waypoints = []
            
            # for each pose, get the pose and convert to kinova pose
            for j in range(pose_num, num_poses+pose_num):
                pose = poses["pose" + str(j)]

                msg = PoseStamped()
                msg.header.frame_id = "board_link"
                msg.pose.position.x = pose["position"]["x"]
                msg.pose.position.y = pose["position"]["y"]
                msg.pose.position.z = pose["position"]["z"]
                msg.pose.orientation.x = pose["orientation"]["x"]
                msg.pose.orientation.y = pose["orientation"]["y"]
                msg.pose.orientation.z = pose["orientation"]["z"]
                msg.pose.orientation.w = pose["orientation"]["w"]

                # convert to base_link frame
                msg_in_base = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

                # convert to kinova_pose
                kp = get_kinovapose_from_pose_stamped(msg_in_base)

                waypoints.append(kp)

            pose_num += num_poses

            print('gripper closed')
            self.arm.execute_gripper_command(gripper_angle)
            success = self.arm.traverse_waypoints(waypoints)

        print('first round done')

        # round 2
        # TODO: check the second round of wind cable if its correct
        pose_num = 1
        for i in range(1, 4):
            gripper_angle = rospy.get_param("~wind_poses/traj" + str(i)+"/gripper")
            
            poses = rospy.get_param("~wind_poses/traj" + str(i)+"/poses")
            
            # from poses dict, get number of poses
            num_poses = len(poses)

            waypoints = []
            
            # for each pose, get the pose and convert to kinova pose
            for j in range(pose_num, num_poses+pose_num):
                pose = poses["pose" + str(j)]

                msg = PoseStamped()
                msg.header.frame_id = "board_link"
                msg.pose.position.x = pose["position"]["x"]
                msg.pose.position.y = pose["position"]["y"]
                msg.pose.position.z = pose["position"]["z"]
                msg.pose.orientation.x = pose["orientation"]["x"]
                msg.pose.orientation.y = pose["orientation"]["y"]
                msg.pose.orientation.z = pose["orientation"]["z"]
                msg.pose.orientation.w = pose["orientation"]["w"]

                # convert to base_link frame
                msg_in_base = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

                # convert to kinova_pose
                kp = get_kinovapose_from_pose_stamped(msg_in_base)

                waypoints.append(kp)

            pose_num += num_poses

            print('gripper closed')
            self.arm.execute_gripper_command(gripper_angle)
            success = self.arm.traverse_waypoints(waypoints)

        # sliding to tip of the cable
        # TODO: change the end pose to the correct one
        # gripper_angle = rospy.get_param("~wind_poses/placing_traj/gripper")
        # self.arm.execute_gripper_command(gripper_angle) 
        # pose = rospy.get_param("~wind_poses/placing_traj/poses/pose10") # msg = PoseStamped()
        # msg.header.frame_id = "board_link"
        # msg.pose.position.x = pose["position"]["x"]
        # msg.pose.position.y = pose["position"]["y"]
        # msg.pose.position.z = pose["position"]["z"]
        # msg.pose.orientation.x = pose["orientation"]["x"]
        # msg.pose.orientation.y = pose["orientation"]["y"]
        # msg.pose.orientation.z = pose["orientation"]["z"]
        # msg.pose.orientation.w = pose["orientation"]["w"]

        # # convert to base_link frame
        # msg_in_base = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

        # # convert to kinova_pose
        # kp = get_kinovapose_from_pose_stamped(msg_in_base)

        # success = self.arm.send_cartesian_pose(kp)

        # pose = rospy.get_param("~wind_poses/placing_traj/poses/pose11")

        # msg = PoseStamped()
        # msg.header.frame_id = "board_link"
        # msg.pose.position.x = pose["position"]["x"]
        # msg.pose.position.y = pose["position"]["y"]
        # msg.pose.position.z = pose["position"]["z"]
        # msg.pose.orientation.x = pose["orientation"]["x"]
        # msg.pose.orientation.y = pose["orientation"]["y"]
        # msg.pose.orientation.z = pose["orientation"]["z"]
        # msg.pose.orientation.w = pose["orientation"]["w"]

        # # convert to base_link frame
        # msg_in_base = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

        # # convert to kinova_pose
        # kp = get_kinovapose_from_pose_stamped(msg_in_base)

        # success = self.arm.send_cartesian_pose(kp)
        # self.arm.execute_gripper_command(0.2) 
        # self.arm.execute_gripper_command(0.4) 
        # self.arm.execute_gripper_command(0.2) 
        return success
    
    def pick_probe_from_holder(self):
        
        # # go to the probe pick perceive position above the holder
        self.arm.execute_gripper_command(0.0)

        perceive_board_pose = rospy.get_param("~perceive_board_pose")
        perceive_board_pose = get_kinovapose_from_list(perceive_board_pose)
        success = self.arm.send_cartesian_pose(perceive_board_pose)

        safe_pose_after_probe_placement = rospy.get_param("~probe_action_poses/safe_pose_after_probe_placement")

        safe_pose_after_probe_placement_kp = get_kinovapose_from_list(safe_pose_after_probe_placement)

        rospy.loginfo("[probe_action] moving to pose before probe grasping")
        success = self.arm.send_cartesian_pose(safe_pose_after_probe_placement_kp, max_lin_vel=0.05)

        probe_place_in_holder_pose = rospy.get_param("~probe_action_poses/probe_place_in_holder_pose")

        probe_place_in_holder_pose_kp = get_kinovapose_from_list(probe_place_in_holder_pose)

        rospy.loginfo("[probe_action] moving to probe place holder position")

        success = self.arm.send_cartesian_pose(probe_place_in_holder_pose_kp, max_lin_vel=0.05)
        if not success:
            rospy.logerr("[probe_action] Failed to move to the probe place holder position")
            return False

        probe_place_in_holder_pose_kp.x += 0.001
        success = self.arm.send_cartesian_pose(probe_place_in_holder_pose_kp, max_lin_vel=0.05)
        if not success:
            rospy.logerr("[probe_action] Failed to move to the probe place holder position")
            return False
        # close the gripper
        self.arm.execute_gripper_command(1.0)
        
        # move up a bit
        probe_place_in_holder_pose_kp.z += 0.1

        success = self.arm.send_cartesian_pose(probe_place_in_holder_pose_kp)

        if not success:
            rospy.logerr("[probe_action] Failed to move up the probe")
            return False
        
        rospy.loginfo("[probe_action] probe picked")

        return True

    def tuck_probe_into_board(self) -> bool:
        # TODO: implement with velocity control
        # get the probe initial position from tf
        msg = PoseStamped()
        msg.header.frame_id = "probe_initial_link"
        msg.header.stamp = rospy.Time(0)
        # TODO: check if this offset is correct wrt placing in holder
        probe_initial_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True, offset=[0, 0, -math.pi/2])

        probe_initial_pose_kp = self.transform_utils.transform_pose_frame_name(reference_frame_name="probe_initial_link",
                                                                      target_frame_name="base_link",
                                                                      offset_linear=[-0.05, 0.00, 0.08],
                                                                      offset_rotation_euler=[math.pi, 0.0, -math.pi/2])

        # send the probe initial position to the arm
        rospy.loginfo("[probe_action] moving to probe initial position")
        success = self.arm.send_cartesian_pose(probe_initial_pose_kp)

        if not success:
            rospy.logerr("[probe_action] Failed to move to the probe initial position")
            return False
        
        rospy.loginfo('[probe_action] reached probe initial position')


        # use velocity control to move the probe down
        rospy.loginfo("[probe_action] moving down the probe")
        # success = self.arm.move_down_with_caution(force_threshold=[4,4,1.75], velocity=0.005, tool_z_thresh=0.10, retract_dist=0.008)

        probe_initial_pose_kp.z = 0.1173

        success = self.arm.send_cartesian_pose(probe_initial_pose_kp, max_lin_vel=0.01)

        if not success:
            rospy.logerr("[probe_action] Failed to move down the probe")
            return False
        
        rospy.loginfo('[probe_action] moved down the probe')
        

        # move the probe back in x direction for 4cm
        success = self.arm.move_with_velocity(-0.025, 3, 'y')

        if not success:
            rospy.logerr("Failed to move back the probe")
            return False

        self.arm.execute_gripper_command(0.6) # open the gripper

        # move the arm up in z axis
        success = self.arm.move_with_velocity(0.03, 3, 'z')
        return True

    def run_visual_servoing(self, vs_target_fn, run=True):
        stop = False
        while not rospy.is_shutdown():
            if self.image is None:
                rospy.loginfo('waiting for image')
                self.loop_rate.sleep()
                continue
            msg = kortex_driver.msg.TwistCommand()
            msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
            x_error, y_error = vs_target_fn(True)
            if x_error is None:
                print('none')
                msg.twist.linear_x = 0.0
            if y_error is None:
                print('none')
                msg.twist.linear_y = 0.0
            if x_error is not None:
                rospy.loginfo('X Error: %.2f' % (x_error))
                if x_error < 0:
                    msg.twist.linear_x = -0.005
                if x_error > 0:
                    msg.twist.linear_x = 0.005
                if abs(x_error) < 40:
                    msg.twist.linear_x = 0.0
                elif abs(x_error) < 50:
                    msg.twist.linear_x *= 0.5

            if y_error is not None:
                rospy.loginfo('Y Error: %.2f' % (y_error))
                if y_error < 0:
                    msg.twist.linear_y = -0.005
                if y_error > 0:
                    msg.twist.linear_y = 0.005
                if abs(y_error) < 3:
                    msg.twist.linear_y = 0.0
                elif abs(y_error) < 10:
                    msg.twist.linear_y *= 0.5
            if run:
                self.cart_vel_pub.publish(msg)
                if msg.twist.linear_x == 0.0 and msg.twist.linear_y == 0 and x_error is not None and y_error is not None:
                    break
            self.loop_rate.sleep()
        msg = kortex_driver.msg.TwistCommand()
        msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        self.cart_vel_pub.publish(msg)

    def detect_wind_cable(self, save_image=False):

        if save_image:
            self.save_debug_image()

        # parameters
        circularity_threshold_min = 0.1
        circularity_threshold_max = 0.5
        contours_area_threshold_min = 5000
        contours_area_threshold_max = 10000

        # draw a rectangle on the image from the center of the image
        x_axis_right = 300
        x_axis_left = 300
        y_axis_top = 10
        y_axis_bottom = self.image.shape[0] # y_axis_bottom is height of the image

        # crop the ROI from the image with the rectangle
        roi = self.image[self.image.shape[0] // 2 - y_axis_top:self.image.shape[0] // 2 + y_axis_bottom,
                    self.image.shape[1] // 2 - x_axis_left:self.image.shape[1] // 2 + x_axis_right]

        # draw a white line on the bottom of the image
        cv2.line(roi, (0, roi.shape[0] - 1), (roi.shape[1],
                roi.shape[0] - 1), (255, 255, 255), 2)

        # draw a white line on the top of the image
        cv2.line(roi, (0, 0), (roi.shape[1], 0), (255, 255, 255), 2)

        roi_copy = roi.copy()
        roi_copy_2 = roi.copy()

        # convert the image to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # apply gaussian blur to the image
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # otsu thresholding
        ret, thresh = cv2.threshold(
            blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # apply canny edge detection
        canny = cv2.Canny(thresh, 50, 150)
        # find the contours
        contours, _ = cv2.findContours(
            canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # draw the contours on the image
        # cv2.drawContours(roi_copy, contours, -1, (0, 255, 0), 2)

        # cv2.imshow("Contours", roi_copy)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # filter out black contours
        filtered_contours = []
        for contour in contours:

            # Calculate area and perimeter of the contour
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # filter out small contours
            if area < contours_area_threshold_min or area > contours_area_threshold_max:
                continue

            # print("Area: {}".format(area))

            # Calculate circularity of the contour
            circularity = (4 * np.pi * area) / (perimeter ** 2)

            # print("Circularity: {}".format(circularity))

            if circularity < circularity_threshold_min or circularity > circularity_threshold_max:
                continue
            
            # draw contours on the image
            # cv2.drawContours(roi_copy_2, [contour], -1, (0, 255, 0), 2)
            # cv2.imshow("Contours", roi_copy_2)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            filtered_contours.append(contour)

        print("Number of filtered contours: {}".format(len(filtered_contours)))
        
        # draw a horizontal line in the middle of the image
        horizontal_line = [(0, roi_copy_2.shape[0] // 2),
                           (roi_copy_2.shape[1], roi_copy_2.shape[0] // 2)]
        cv2.line(roi_copy_2, horizontal_line[0], horizontal_line[1], (0, 0, 255), 2)

        # draw a vertical line in the middle of the image
        vertical_line = [(roi_copy_2.shape[1] // 2, 0),
                        (roi_copy_2.shape[1] // 2, roi_copy_2.shape[0])]
        cv2.line(roi_copy_2, vertical_line[ 0], vertical_line[1], (0, 0, 255), 2)

        # NOTE: it should only be one contour
        if len(filtered_contours) > 1:
            print("More than one contour found!")
            print("TODO 1: failure recovery mechanism")
            return (None, None)

        elif len(filtered_contours) == 1:
            # get the contour points
            contour_points = filtered_contours[0]

            # get the bottom most point of the contour
            bottom_most_point = contour_points[contour_points[:, :, 1].argmax()][0]

            # draw a circle on the bottom most point
            cv2.circle(
                roi_copy_2, (bottom_most_point[0], bottom_most_point[1]), 5, (0, 0, 255), -1)

            error = (roi.shape[1] // 2) - bottom_most_point[0]
            print("Error: {}".format(error))

            # print the error on the image
            cv2.putText(roi_copy_2, "Error: {}".format(error), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # draw the error line on the image from the centroid to the vertical line
            error_line = [bottom_most_point, (roi.shape[1] // 2, bottom_most_point[1])]
            cv2.line(roi_copy_2, error_line[0], error_line[1], (255, 0, 0), 2)

            self.img_pub.publish(self.bridge.cv2_to_imgmsg(roi_copy_2, "bgr8"))

            return (error, None)

        else:
            print("No contour found!")
            return (None, None)
        
    def save_debug_image(self):
        config_path = os.path.join(os.path.dirname(__file__), '../..', 'images')
        
        # get the current date and time
        now = datetime.datetime.now()

        if self.image is not None:
            cv2.imwrite(os.path.join(config_path, 'debug_image_{}.png'.format(now)), self.image)
