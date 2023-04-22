#!/usr/bin/env python3

import rospy
from robothon2023.abstract_action import AbstractAction
from geometry_msgs.msg import PoseStamped, Quaternion
import geometry_msgs.msg
import sensor_msgs.msg
from sensor_msgs.msg import Image
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from utils.kinova_pose import KinovaPose, get_kinovapose_from_list, get_kinovapose_from_pose_stamped
from utils.perception_utils import dilate, erode, get_uppermost_contour, detect_door_circle

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
        self.current_force_z = []
        self.current_height = None
        self.loop_rate = rospy.Rate(10)
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', kortex_driver.msg.TwistCommand, queue_size=1)
        self.base_feedback_sub = rospy.Subscriber('/my_gen3/base_feedback', kortex_driver.msg.BaseCyclic_Feedback, self.base_feedback_cb)
        self.door_knob_pose_pub = rospy.Publisher("/door_knob_pose", PoseStamped, queue_size=1)
        self.probe_cable_dir_debug_pub = rospy.Publisher('/probe_cable_dir_debug', Image, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', sensor_msgs.msg.Image, self.image_cb)
        self.debug = rospy.get_param("~debug", False)
        self.bridge = cv_bridge.CvBridge()
        self.transform_utils = TransformUtils()
        
    def base_feedback_cb(self, msg):
        self.current_force_z.append(msg.base.tool_external_wrench_force_z)
        if len(self.current_force_z) > 25:
            self.current_force_z.pop(0)
        self.current_height = msg.base.tool_pose_z

    def image_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    def pre_perceive(self) -> bool:
        print ("in pre perceive")        
        
        return True

    def act(self) -> bool:
        # pick the probe from the box and place it in the holder
        # success = self.place_probe_in_holder()

        # success = True
        # if not success:
        #     rospy.logerr("[probe_action] Failed to place the probe in the holder")
        #     return False
        
        # open the door with magnet
        # success = self.open_door()

        # if not success:
        #     rospy.logerr("[probe_action] Failed to open the door")
        #     return False
        
        success = self.open_door_with_trajactroy()
        if not success:
            rospy.logerr("[probe_action] Failed to open the door")
            return False
        
        
        # pick the probe from the holder
        # success = self.pick_probe_from_holder()
        
        # probe the circuit
        # success = self.probe_circuit()

        # if not success:
        #     rospy.logerr("[probe_action] Failed to probe the circuit")
        #     return False
        
        # place the probe somewhere safe
        # success = self.place_probe_safe()

        # if not success:
        #     rospy.logerr("[probe_action] Failed to place the probe somewhere safe")
        #     return False
        
        probed = False
        retries = 0
        while not probed:
            self.run_visual_servoing(self.get_probe_point_error, target_height=0.3)
            probed = self.move_down_and_probe()
            retries += 1
            if retries > 5:
                break
        success = probed

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
        

#### door opening without magnet


        # go to the door knob position
        # get the door knob position from tf
        msg = PoseStamped()
        msg.header.frame_id = "door_knob_link"
        msg.header.stamp = rospy.Time(0)
        # msg.pose.position.x -= 0.015

        door_knob_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True, offset=[0.0, 0.0, -math.pi/2])

        # convert the door knob pose to a kinova pose
        door_knob_kinova_pose = get_kinovapose_from_pose_stamped(door_knob_pose)

        # move up a bit
        door_knob_kinova_pose.z += 0.022 + 0.05 # adding 5 cm for approach

        # send the door knob pose to the arm
        print("[probe_action] moving to door knob position")
        print("[probe_action] door knob pose: {}".format(door_knob_kinova_pose))

        success = self.arm.send_cartesian_pose(door_knob_kinova_pose)
        # return False
        rospy.sleep(1.0) # wait for the arm to settle for proper force sensing

        if not success:
            rospy.logerr("Failed to move to the door knob position")
            return False



        # velocity mode to approach the door knob
        success = self.arm.move_down_with_caution(velocity=0.01, force_threshold=[4,4,2.0])

        if not success:
            rospy.logerr("Failed to move down the arm")
            return False
        print("[probe_action] reached door knob position")



        # close the gripper
        success = self.arm.execute_gripper_command(0.75) # 0.75 closed 
        if not success:
            rospy.logerr("Failed to open the gripper")
            return False
        

        # # linear_vel_z = 0.0025
        # vel = 0.01
        # arc_radius = 0.07
        # angle = 45
        # angular_velocity = vel/arc_radius

        # angular_velocity *= 0.05 # 20% of the linear velocity
        # # angle = 25.0
        # # # w.r.t board link
        # # linear_vel_x = vel*math.cos(math.radians(angle))
        # # linear_vel_z = vel*math.sin(math.radians(angle))


        door_open_twist = kortex_driver.msg.TwistCommand()
        door_open_twist.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL


        # linear_velocity_x = abs(vel*math.cos(math.degrees(angle)))
        # linear_velocity_z = abs(vel*math.sin(math.degrees(angle)))
        # angular_velocity_y = abs(angular_velocity)

        # linear_velocity_x *= 0.95
        # linear_velocity_z *= 1.1

        # switch = -1
        # kill_z = 0

        angle = 0
        radius = 0.07
        radius_y = 0.068
        radius_z = 0.065
        arc_length = (2 * math.pi * radius) /4
        travel_time = 1
        angular_velocity_door = arc_length / travel_time
        angular_velocity_door *= 10
        travel_time = 15

        for t in range(travel_time):
            
            print("time==>: ", t, "s")
            # angle_feedback = np.rad2deg(angular_velocity_door * t)

            print("angluar velocity of the door : ", angular_velocity_door, "rad/s")
            # print("angle_feedback of the motion : ", angle_feedback)
            # angle = np.deg2rad(angle_feedback)

            angle += np.deg2rad(90/travel_time)

            #ellipse motion

            velocity_door_ellipse = angular_velocity_door * math.sqrt( radius_z**2 * math.cos(angle)**2 +
                                                                        radius_y**2 * math.sin(angle)**2)

            omega_ =  (angular_velocity_door /  math.sqrt( radius_z**2 * math.cos(angle)**2 +
                                                        radius_y**2 * math.sin(angle)**2))


            # y and z calculation
            y = omega_ * radius_y * math.cos(angle)
            z = omega_ * radius_z * math.sin(angle)

            #theta calculation
            theta = omega_ * t  


            # velocity in y and z calculation
            vy = -radius_y * math.sin(angle) * velocity_door_ellipse
            vz = radius_z * math.cos(angle) * velocity_door_ellipse

            # assignment step

            linear_velocity_y = vy
            linear_velocity_z = vz


            print("++++++++++++++++++++++++++++++")
            # printing step 
            print("linear_velocity_y: ", linear_velocity_y, "m/s")
            print("linear_velocity_z: ", linear_velocity_z, "m/s")

            print("++++++++++++++++++++++++++++++"+"\n")

            # publishing step
            door_open_twist.twist.linear_y = -abs(linear_velocity_y) # negative sign because the tool frame is facing the downward direction
            door_open_twist.twist.linear_z = -abs(linear_velocity_z) 
            # door_open_twist.twist.angular_x = angular_velocity_x
            self.cart_vel_pub.publish(door_open_twist)

            rospy.sleep(1)

        msg = kortex_driver.msg.TwistCommand()

        msg.twist.linear_z = 0.0
        msg.twist.linear_x = 0.0
        msg.twist.linear_y = 0.0
        msg.twist.angular_x = 0.0
        msg.twist.angular_y = 0.0
        msg.twist.angular_z = 0.0

        self.cart_vel_pub.publish(msg)

        self.arm.clear_faults()
        self.arm.subscribe_to_a_robot_notification()

### door opening without magnet

        # get current pose of the arm
        current_pose = self.arm.get_current_pose()

        # go up by 5cm
        current_pose.z += 0.1

        success = self.arm.send_cartesian_pose(current_pose)

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

    def open_door_with_trajactroy(self): # working code 

        msg = PoseStamped()
        msg.header.frame_id = "door_knob_link"
        msg.header.stamp = rospy.Time(0)
        # msg.pose.position.x -= 0.015

        door_knob_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True, offset=[0.0, 0.0, -math.pi/2])

        # convert the door knob pose to a kinova pose
        door_knob_kinova_pose = get_kinovapose_from_pose_stamped(door_knob_pose)

        # move up a bit
        door_knob_kinova_pose.z += 0.022 + 0.05 # adding 5 cm for approach

        # send the door knob pose to the arm
        print("[probe_action] moving to door knob position")
        print("[probe_action] door knob pose: {}".format(door_knob_kinova_pose))

        success = self.arm.send_cartesian_pose(door_knob_kinova_pose)
        # return False
        rospy.sleep(1.0) # wait for the arm to settle for proper force sensing

        if not success:
            rospy.logerr("Failed to move to the door knob position")
            return False

        rospy.loginfo("Visual servoing to door knob")
        self.run_visual_servoing(self.get_door_knob_error, target_height = 0.11)

        # velocity mode to approach the door knob
        success = self.arm.move_down_with_caution(velocity=0.01, force_threshold=[4,4,2.0])

        if not success:
            rospy.logerr("Failed to move down the arm")
            return False
        print("[probe_action] reached door knob position")

        # close the gripper
        success = self.arm.execute_gripper_command(0.75) # 0.75 closed 
        if not success:
            rospy.logerr("Failed to open the gripper")
            return False
        

        pose_list = self.get_trajactory_poses(num_poses=8)

        success = self.arm.traverse_waypoints(pose_list)

        if not success:
            rospy.logerr("Failed to reach desired pose")
            return False
        return success

    def push_door(self):  # push door is not cuurently used
            # move arm above door knob position 

        msg = PoseStamped()
        msg.header.frame_id = "door_knob_link"
        msg.header.stamp = rospy.Time(0)
        msg = PoseStamped()
        msg.header.frame_id = "door_knob_link"
        msg.header.stamp = rospy.Time(0)

        msg.pose.position.x -= 0.075
        msg.pose.position.z += 0.30
        msg.pose.position.x -= 0.075
        msg.pose.position.z += 0.30

        door_knob_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True)
        door_knob_pose = self.transform_utils.transformed_pose_with_retries(msg, "base_link", execute_arm=True)

        print("Door knob pose:  ")
        print(door_knob_pose)
        self.door_knob_pose_pub.publish(door_knob_pose)
        print("Door knob pose:  ")
        print(door_knob_pose)
        self.door_knob_pose_pub.publish(door_knob_pose)

        rospy.sleep(5)
        rospy.sleep(5)

        success = self.arm.execute_gripper_command(1.0)
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

    def get_door_knob_error(self):
        '''
        Get error for aligning with door knob using visual servoing
        '''
        min_x = 280
        max_x = 1000
        min_y = 400
        max_y = 720
        # at height 0.11
        target_x = 362
        target_y = 299
        image = self.image[min_y:max_y, min_x:max_x]

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.Canny(gray, 80, 200)
        gray = dilate(gray)
        gray = erode(gray)
        circles = detect_door_circle(gray)
        if circles is not None:
            if len(circles) > 1:
                return None, None
            elif len(circles) == 1:
                cc = circles[0]
                cx, cy, r = cc[0], cc[1], cc[2]
                # Draw the circumference of the circle.
                cv2.circle(image, (cx, cy), r, (0, 255, 0), 2)
                error_x = target_x - cx
                error_y = target_y - cy
        else:
            error_x = None
            error_y = None
        cv2.imshow("image", image)
        cv2.waitKey(1)
        return error_x, error_y


    def get_orange_mask(self, img):
        lower = np.array([80, 120, 140])
        upper = np.array([120, 255, 255])
        mask = cv2.inRange(img, lower, upper)
        mask = dilate(mask)
        mask = erode(mask)
        return mask

    def get_probe_point_error(self):
        '''
        Get error for circuit probe point for visual servoing
        '''
        ## set at height of 0.3 m (i.e tool_pose_z = 0.3)
        target_x = 634
        target_y = 464
        if self.image is None:
            return None, None
        (height, width, c) = self.image.shape
        start_y = int(height / 4)
        start_x = int(width * 0.4)
        img = self.image[start_y:int(height - height/4), start_x:int(width - width/4), :]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = self.get_orange_mask(hsv)
        cx, cy = get_uppermost_contour(mask)
        if cx is None:
            return None, None
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), 2)
        cv2.imshow('debug', img)
        cv2.waitKey(1)
        error_x = target_x - (cx + start_x)
        error_y = target_y - (cy + start_y)
        return error_x, error_y

    def run_visual_servoing(self, vs_target_fn, target_height):
        success = False
        rospy.loginfo("Moving to correct height")
        while not rospy.is_shutdown():
            # we need to be at 0.3 height to do VS
            height_error = self.current_height - target_height
            msg = kortex_driver.msg.TwistCommand()
            msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
            if height_error > 0.001:
                msg.twist.linear_z = 0.02
            elif height_error < -0.001:
                msg.twist.linear_z = -0.02
            else:
                break
            self.cart_vel_pub.publish(msg)
            self.loop_rate.sleep()

        rospy.loginfo("visual servoing")
        stop = False
        while not rospy.is_shutdown():
            if len(self.current_force_z) < 20:
                self.loop_rate.sleep()
                continue
            msg = kortex_driver.msg.TwistCommand()
            msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
            error_x, error_y = vs_target_fn()
            if error_x is None:
                msg.twist.linear_x = 0.0
            else:
                rospy.loginfo('%d, %d' % (error_x, error_y))
                if error_x < 0:
                    msg.twist.linear_x = -0.005
                if error_x > 0:
                    msg.twist.linear_x = 0.005
                if abs(error_x) < 3:
                    msg.twist.linear_x = 0.0
                elif abs(error_x) < 10:
                    msg.twist.linear_x *= 0.5

                if error_y < 0:
                    msg.twist.linear_y = -0.005
                if error_y > 0:
                    msg.twist.linear_y = 0.005
                if abs(error_y) < 3:
                    msg.twist.linear_y = 0.0
                elif abs(error_y) < 10:
                    msg.twist.linear_y *= 0.5
            self.cart_vel_pub.publish(msg)
            if msg.twist.linear_x == 0.0 and msg.twist.linear_y == 0 and error_x is not None:
                break
            self.loop_rate.sleep()
        msg = kortex_driver.msg.TwistCommand()
        msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        msg.twist.linear_x = 0.0
        msg.twist.linear_y = 0.0
        self.cart_vel_pub.publish(msg)

    def move_down_and_probe(self):
        '''
        Once we have aligned with the probe circuit point, go down
        '''
        #### Go down fast
        self.current_force_z = []
        stop = False
        while not rospy.is_shutdown():
            if len(self.current_force_z) < 20:
                self.loop_rate.sleep()
                continue
            msg = kortex_driver.msg.TwistCommand()
            msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
            msg.twist.linear_z = 0.02
            if abs(np.mean(self.current_force_z) - self.current_force_z[-1]) > 3.0:
                stop = True
                msg.twist.linear_z = 0.0
            if self.current_height < 0.235: # just above the hole
                stop = True
                msg.twist.linear_z = 0.0
            self.cart_vel_pub.publish(msg)
            if stop:
                break
            self.loop_rate.sleep()

        #### Go down slowly
        self.current_force_z = []
        stop = False
        while not rospy.is_shutdown():
            if len(self.current_force_z) < 20:
                self.loop_rate.sleep()
                continue
            msg = kortex_driver.msg.TwistCommand()
            msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
            msg.twist.linear_z = 0.005
            if abs(np.mean(self.current_force_z) - self.current_force_z[-1]) > 5.0:
                rospy.loginfo("Force difference threshold reached")
                stop = True
                msg.twist.linear_z = 0.0
            if self.current_height < 0.185: # already inside
                stop = True
                msg.twist.linear_z = 0.0
            self.cart_vel_pub.publish(msg)
            if stop:
                break
            self.loop_rate.sleep()
        probed = False
        if self.current_height < 0.195:
            probed = True

        #### Go up
        msg = kortex_driver.msg.TwistCommand()
        msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
        msg.twist.linear_z = -0.01
        for idx in range(50):
            self.cart_vel_pub.publish(msg)
            self.loop_rate.sleep()
        msg.reference_frame = kortex_driver.msg.CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        msg.twist.linear_z = 0.0
        self.cart_vel_pub.publish(msg)
        self.loop_rate.sleep()
        return probed

    def get_kinova_pose(self,pose_name):

        pose = rospy.get_param("~"+ pose_name)

        sample = PoseStamped()
        sample.header.stamp = rospy.Time(0)
        sample.header.frame_id = "board_link"
        sample.pose.position.x = pose['pose']['position']['x']
        sample.pose.position.y = pose['pose']['position']['y']
        sample.pose.position.z = pose['pose']['position']['z']
        sample.pose.orientation.x = pose['pose']['orientation']['x']
        sample.pose.orientation.y = pose['pose']['orientation']['y']
        sample.pose.orientation.z = pose['pose']['orientation']['z']
        sample.pose.orientation.w = pose['pose']['orientation']['w']

        pose_in_base_link = self.transform_utils.transformed_pose_with_retries(sample, "base_link", 3)

        pose_in_kinova_pose = get_kinovapose_from_pose_stamped(pose_in_base_link)

        return pose_in_kinova_pose

    def get_trajactory_poses(self,num_poses):
        """
        get poses

        input: num_poses: number of poses to get
        return: list of poses
        """

        pose_list = []
        for i in range(num_poses):
            pose_name = "pose" + str(i+1)
            pose = self.get_kinova_pose(pose_name)
            pose_list.append(pose)
        return pose_list

