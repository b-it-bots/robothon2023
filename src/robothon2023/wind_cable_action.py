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

class WindCableAction(AbstractAction):
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:
        super().__init__(arm, transform_utils)
        self.debug = rospy.get_param("~debug", False)
        self.img_sub = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_cb)
        self.img_pub = rospy.Publisher(
            '/visual_servoing_debug_img', Image, queue_size=10)
        self.image = None

    def pre_perceive(self) -> bool:
        print ("in pre perceive")        
        
        return True

    def act(self) -> bool:
        # success = self.transform_poses()
        self.run_visual_servoing(self.detect_wind_cable, run=True)
        success = self.wind_cable()

        # success = self.test_transform()

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
    
    def transform_poses(self):

        wind_cable_poses = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses_board/p" + str(i))
            kp = get_kinovapose_from_list(pose)
            pose = kp.to_pose_stamped()
            pose_in_board = self.transform_utils.transformed_pose_with_retries(pose, "board_link")
            wind_cable_poses.append(pose_in_board)
        
        wind_cable_poses2 = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses_board2/p" + str(i))
            kp = get_kinovapose_from_list(pose)
            pose = kp.to_pose_stamped()
            pose_in_board = self.transform_utils.transformed_pose_with_retries(pose, "board_link")
            wind_cable_poses2.append(pose_in_board)

        print('length of wind_cable_poses: ', len(wind_cable_poses))
        print('length of wind_cable_poses2: ', len(wind_cable_poses2))

        # get relative path to the config folder
        config_path = os.path.join(os.path.dirname(__file__), '../..', 'config')

        # write the poses to a yaml file
        with open(config_path+"/wind_cable_poses.yaml", "w+") as f:
            f.write("wind_cable_poses:\n")
            for i in range(1, 11):
                print('i: ', i)
                p: PoseStamped = wind_cable_poses[i-1]
                # convert orientation to euler angles using tf
                euler = tf.transformations.euler_from_quaternion([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])
                f.write(f"  p{str(i)} : [{p.pose.position.x}, {p.pose.position.y}, {p.pose.position.z}, {euler[0]}, {euler[1]}, {euler[2]}]\n")
            f.write("wind_cable_poses2:\n")
            for i in range(1, 11):
                print('i: ', i)
                p: PoseStamped = wind_cable_poses2[i-1]
                # convert orientation to euler angles using tf
                euler = tf.transformations.euler_from_quaternion([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])
                f.write(f"  p{str(i)} : [{p.pose.position.x}, {p.pose.position.y}, {p.pose.position.z}, {euler[0]}, {euler[1]}, {euler[2]}]\n")

        return True
    
    def wind_cable(self) -> bool:
        
        wind_cable_kinova_poses = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses/p" + str(i))

            # convert the euler angles to quaternion
            euler = [pose[3], pose[4], pose[5]]
            quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

            msg = PoseStamped()
            msg.header.frame_id = "board_link"
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            msg.pose.orientation = Quaternion(*quaternion)

            # convert to base_link frame
            msg_in_base = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

            # convert to kinova_pose
            kp = get_kinovapose_from_pose_stamped(msg_in_base)

            wind_cable_kinova_poses.append(kp)

        success = self.arm.send_cartesian_pose(wind_cable_kinova_poses[0])

        # close gripper
        self.arm.execute_gripper_command(0.8)

        success = self.arm.traverse_waypoints(wind_cable_kinova_poses)

        print('first round done')

        wind_cable_kinova_poses2 = []
        for i in range(1, 11):
            pose = rospy.get_param("~wind_cable_poses2/p" + str(i))

            # convert the euler angles to quaternion
            euler = [pose[3], pose[4], pose[5]]
            quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

            msg = PoseStamped()
            msg.header.frame_id = "board_link"
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            msg.pose.orientation = Quaternion(*quaternion)

            # convert to base_link frame
            msg_in_base = self.transform_utils.transformed_pose_with_retries(msg, "base_link")

            # convert to kinova_pose
            kp = get_kinovapose_from_pose_stamped(msg_in_base)

            wind_cable_kinova_poses2.append(kp)

        print('starting second round')

        success = self.arm.traverse_waypoints(wind_cable_kinova_poses2)

        # go out of the way to keep the probe safe
        # TODO: figure out which way to go
    
    def run_visual_servoing(self, vs_target_fn, run=True):
        stop = False
        while not rospy.is_shutdown():
            if self.image is None:
                rospy.loginfo('waiting for image')
                self.loop_rate.sleep()
                continue
            msg = kortex_driver.msg.TwistCommand()
            msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
            x_error, y_error = vs_target_fn()
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
                if abs(x_error) < 10:
                    msg.twist.linear_x = 0.0
                elif abs(x_error) < 20:
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

    def detect_wind_cable(self) -> tuple(float, float):

        # parameters
        image_idx = 7  # works for index 0, 7, 9
        circularity_threshold_min = 0.1
        circularity_threshold_max = 0.5
        contours_area_threshold_min = 5000
        contours_area_threshold_max = 8000
        black_color_threshold = 60

        # draw a rectangle on the image from the center of the image
        x_axis_right = 300
        x_axis_left = 300
        y_axis_top = 200
        # y_axis_bottom is height of the image
        y_axis_bottom = self.image.shape[0]

        # crop the ROI from the image with the rectangle
        roi = self.image[self.image.shape[0] // 2 - y_axis_top:self.image.shape[0] // 2 + y_axis_bottom,
                    self.image.shape[1] // 2 - x_axis_left:self.image.shape[1] // 2 + x_axis_right]


        # draw a white rectangle on the border of the image
        cv2.rectangle(roi, (0, 0), (roi.shape[1], roi.shape[0]), (255, 255, 255), 2)

        roi_copy = roi.copy()
        roi_copy_2 = roi.copy()

        # canny edge detection on the ROI
        # convert the image to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # apply gaussian blur to the image
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # apply canny edge detection
        canny = cv2.Canny(blur, 50, 150)
        # find the contours
        contours, _ = cv2.findContours(
            canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # filter out black contours
        filtered_contours = []
        mean_colors = []
        circularities = []
        for contour in contours:

            # Calculate area and perimeter of the contour
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # filter out small contours
            if area < contours_area_threshold_min or area > contours_area_threshold_max:
                # if area < contours_area_threshold_min:
                continue

            # print("Area: {}".format(area))

            # Calculate circularity of the contour
            circularity = (4 * np.pi * area) / (perimeter ** 2)

            # print("Circularity: {}".format(circularity))

            if circularity < circularity_threshold_min or circularity > circularity_threshold_max:
                continue

            filtered_contours.append(contour)

        # print("Number of filtered contours: {}".format(len(filtered_contours)))

        # draw a horizontal line in the middle of the image
        horizontal_line = [(0, roi_copy_2.shape[0] // 2),
                           (roi_copy_2.shape[1], roi_copy_2.shape[0] // 2)]
        cv2.line(roi_copy_2, horizontal_line[0], horizontal_line[1], (0, 0, 255), 2)

        # draw a vertical line in the middle of the image
        vertical_line = [(roi_copy_2.shape[1] // 2, 0),
                        (roi_copy_2.shape[1] // 2, roi_copy_2.shape[0])]
        cv2.line(roi_copy_2, vertical_line[0], vertical_line[1], (0, 0, 255), 2)

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
            cv2.line(roi_copy_2, error_line[0], error_line[1], (0, 255, 0), 2)

            return (error, None)

        else:
            print("No contour found!")
            return (None, None)