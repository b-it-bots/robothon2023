#!/usr/bin/env python3
#
import rospy
import numpy as np
import kortex_driver.msg
from typing import Type, TypeVar
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from robothon2023.abstract_action import AbstractAction
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import math


class PlugRemoveSlidAction(AbstractAction):
    """
    Assumption: We have approximate pose of the plug .

    - Move arm to the top of the plugs
    - Do visual servoing and adjust arm
    - Open gripper
    - Move arm with velocity control downwards
    - stope when the force is higher 
    - retreate above
    - close gripper
    - move up 
    - go to fixed insert position 
    - Move downwards with velocity control
    - Check force for 2 levels - (board collision and slid force)  needs to be identified
    - Stop if board collision is hit
    - retreat above and random sample x and y pose
    - if slid force is there for some amount of time then collision force comes then open gripper
    - 
    """
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:
        super(PlugRemoveSlidAction, self).__init__(arm, transform_utils)
        self.base_feedback_sub = rospy.Subscriber('/my_gen3/base_feedback', kortex_driver.msg.BaseCyclic_Feedback, self.base_feedback_cb)
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', kortex_driver.msg.TwistCommand, queue_size=1)
        self.img_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_cb)
        self.img_pub = rospy.Publisher('/visual_servoing_debug_img', Image, queue_size=10)
        self.current_force_z = []
        self.loop_rate = rospy.Rate(10.0)
        self.current_force_z = []
        self.bridge = CvBridge()
        self.error = 0.0
        self.error_threshold = 5.0
        self.stop = False
        self.velocity = 0.005
        self.image_queue = []
        self.move_up_done = False
        self.move_down_done = False
        self.close_gripper_done = False

    def base_feedback_cb(self, msg):
        self.current_force_z.append(msg.base.tool_external_wrench_force_z)
        if len(self.current_force_z) > 25:
            self.current_force_z.pop(0)

    def pre_perceive(self) -> bool:
        print ("in pre perceive")

        pre_height_above_button = rospy.get_param("~pre_height_above_button", 0.3)
        kinova_pose = self.transform_utils.transform_pose_frame_name(reference_frame_name="meter_plug_black_link",
                                                                      target_frame_name="base_link",
                                                                      offset_linear=[0.00, 0.00, pre_height_above_button],
                                                                      offset_rotation_euler=[math.pi, 0.0, math.pi/2])

        self.arm.send_cartesian_pose(kinova_pose)
        return True


    def act(self) -> bool:
        #Allign camera 
        self.run_visual_servoing()
        self.move_down_velocity_control()
        self.arm.execute_gripper_command(0.9) #close gripper
        self.move_up_velocity_control()
        self.move_forward()
        pass

    def verify(self) -> bool:
        pass

    def image_cb(self, msg):
        
        # get the image from the message
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # store only the last 20 images
        if len(self.image_queue) > 20:
            self.image_queue.pop(0)
        self.image_queue.append(image)


    def run_visual_servoing(self):
        while not rospy.is_shutdown():
            
            if len(self.image_queue) == 0:
                self.loop_rate.sleep()
                continue
            
            image = self.image_queue[-1]

            circularity_threshold = 0.8
            contours_area_threshold = 100
            black_color_threshold = 60
            
            # find the contours
            # convert the image to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # apply gaussian blur to the image
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            # apply canny edge detection
            canny = cv2.Canny(blur, 50, 150)
            # find the contours
            contours, _ = cv2.findContours(
                canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # draw a horizontal line in the middle of the image
            horizontal_line = [(0, image.shape[0] // 2),
                            (image.shape[1], image.shape[0] // 2)]
            cv2.line(image, horizontal_line[0], horizontal_line[1], (0, 0, 255), 2)

            # draw a vertical line in the middle of the image
            vertical_line = [(image.shape[1] // 2, 0),
                            (image.shape[1] // 2, image.shape[0])]
            cv2.line(image, vertical_line[0], vertical_line[1], (0, 0, 255), 2)

            # filter out black contours
            filtered_contours = []
            mean_colors = []
            circularities = []
            for contour in contours:

                # Calculate area and perimeter of the contour
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)

                # filter out small contours
                if area < contours_area_threshold:
                    continue

                # Calculate circularity of the contour (reference: https://en.wikipedia.org/wiki/Roundness)
                circularity = (4 * np.pi * area) / (perimeter ** 2)

                # create a mask of the contour
                n_mask = np.zeros(image.shape[:2], dtype="uint8")
                cv2.drawContours(n_mask, [contour], 0, (255, 255, 255), -1)

                # get the mean color of the contour using the mask
                mean_color = cv2.mean(image, mask=n_mask)

                # filter out non black contours
                # threshold the circularity value to classify the contour as circular or not
                if mean_color[0] < black_color_threshold and circularity > circularity_threshold:
                    filtered_contours.append(contour)
                    mean_colors.append(mean_color[0])
                    circularities.append(circularity)

            # print("Number of filtered contours: {}".format(len(filtered_contours)))

            # NOTE: it should only be one contour
            # draw the filtered contour one by one on the image
            for i, contour in enumerate(filtered_contours):
                cv2.drawContours(image, [contour], -1, (255, 0, 0), 3)

                # calculate the centroid of the contour
                M = cv2.moments(contour)
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                centroid = (centroid_x, centroid_y)
                cv2.circle(image, centroid, 5, (0, 0, 255), -1)

                # calculate the perpendicular distance between the vertical line and the centroid in the image (y-axis only)
                self.error = (image.shape[1] // 2) - centroid_x
                print("Error: {}".format(self.error))

                # TODO: put a max threshold on error to prevent the arm from moving too much
                # call the arm motion function
                if not self.stop:
                    if abs(self.error) > 2 and abs(self.error) < 300:
                        if self.error > 0:
                            self.move_arm_2D_space(1)
                            rospy.loginfo("Moving arm to the right")
                        else:
                            self.move_arm_2D_space(-1)
                            rospy.loginfo("Moving arm to the left")
                    else:
                        self.move_arm_2D_space(0)
                        continue
                else:
                    self.move_arm_2D_space(0)
                    return True

                # draw the error line on the image from the centroid to the vertical line
                error_line = [centroid, (centroid_x + self.error, centroid_y)]
                cv2.line(image, error_line[0], error_line[1], (0, 255, 0), 2)

                # publish the image
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                # rospy.loginfo("Published a final image!")

    def move_down_velocity_control(self):
        linear_vel_z = rospy.get_param("~linear_vel_z", 0.005)
        force_z_diff_threshold = rospy.get_param("~force_z_diff_threshold", 3.0)
        force_control_loop_rate = rospy.Rate(rospy.get_param("~force_control_loop_rate", 10.0))
        stop = False
        self.current_force_z = []
        num_retries = 0
        while not rospy.is_shutdown():
            if len(self.current_force_z) < 20:
                num_retries += 1
                if num_retries > 100:
                    rospy.logerr("No force measurements received")
                    break
                force_control_loop_rate.sleep()
                continue
            msg = kortex_driver.msg.TwistCommand()
            msg.twist.linear_z = -linear_vel_z
            # print("Force: {}".format(abs(np.mean(self.current_force_z) - self.current_force_z[-1])))
            if abs(np.mean(self.current_force_z) - self.current_force_z[-1]) > force_z_diff_threshold:
                rospy.loginfo("Force difference threshold reached")
                stop = True
                msg.twist.linear_z = 0.0
            self.cart_vel_pub.publish(msg)
            if stop:
                break
            force_control_loop_rate.sleep()
        msg = kortex_driver.msg.TwistCommand()
        msg.twist.linear_z = linear_vel_z
        for idx in range(10):
            self.cart_vel_pub.publish(msg)
            force_control_loop_rate.sleep()
        msg.twist.linear_z = 0.0
        self.cart_vel_pub.publish(msg)
        force_control_loop_rate.sleep()
        rospy.sleep(0.1)
        return True
    

    def move_arm_2D_space(self, direction):

        msg = kortex_driver.msg.TwistCommand()
        if direction == 0:
            rospy.loginfo("Stopped moving")
            msg.twist.linear_x = 0.0
            self.cart_vel_pub.publish(msg)
            self.loop_rate.sleep()
            self.stop = True
            self.cart_vel_pub.publish(msg)
        elif direction == -1 or direction == 1:
            rospy.loginfo("Moving in y direction")
            msg.twist.linear_x = direction * self.velocity
            self.cart_vel_pub.publish(msg)
            self.loop_rate.sleep()
        else:
            rospy.loginfo("Invalid direction")

    def move_up_velocity_control(self):
        rospy.loginfo("Moving up")
        pre_height_above_button = rospy.get_param("~pre_height_above_button", 0.20)
        msg = kortex_driver.msg.TwistCommand()
        for idx in range(40):
            msg.twist.linear_z = 0.01
            self.cart_vel_pub.publish(msg)
            self.loop_rate.sleep()
        msg.twist.linear_z = 0.0
        self.cart_vel_pub.publish(msg)
        self.loop_rate.sleep()
        self.cart_vel_pub.publish(msg)

    def move_forward(self):
        rospy.loginfo("Moving forward")
        msg = kortex_driver.msg.TwistCommand()
        for idx in range(20):
            msg.twist.linear_x = 0.01
            self.cart_vel_pub.publish(msg)
            self.loop_rate.sleep()
        msg.twist.linear_x = 0.0
        self.cart_vel_pub.publish(msg)
        self.loop_rate.sleep()
        self.cart_vel_pub.publish(msg)
