#!/usr/bin/env python3
"""

"""
# -*- encoding: utf-8 -*-

import os
import json
import math

import geometry_msgs.msg
import std_msgs.msg
import rospy
import tf
import numpy as np
from robothon2023.transform_utils import TransformUtils


class PointsOfInterestPublisher(object):
    def __init__(self):
        self.board_poses_queue = []
        self.fixed_board_pose = None

        self.board_pose_sub = rospy.Subscriber('~approximate_board_pose', geometry_msgs.msg.PoseStamped, self.board_pose_cb)
        self.board_pose_pub = rospy.Publisher('~fixed_board_pose', geometry_msgs.msg.PoseStamped, queue_size=1)

        self.event_out_pub = rospy.Publisher('~board_detector_event_out', std_msgs.msg.String, queue_size=1)

        self.num_detections_of_board = rospy.get_param('/task_board_detector/num_detections_of_board')
        board_to_blue_button = rospy.get_param("~board_to_blue_button")
        board_to_red_button = rospy.get_param("~board_to_red_button")
        board_to_slider_start = rospy.get_param("~board_to_slider_start")
        board_to_slider_end = rospy.get_param("~board_to_slider_end")
        board_to_meter_plug_black = rospy.get_param("~board_to_meter_plug_black")
        board_to_meter_plug_red = rospy.get_param("~board_to_meter_plug_red")
        board_to_door_knob = rospy.get_param("~board_to_door_knob")
        board_to_gui = rospy.get_param("~board_to_gui")
        board_to_probe_initial = rospy.get_param("~board_to_probe_initial")
        board_to_wind_cable = rospy.get_param("~board_to_wind_cable")

        self.all_transforms = [board_to_blue_button, board_to_red_button, board_to_slider_start, board_to_slider_end, board_to_meter_plug_black, board_to_meter_plug_red, board_to_door_knob, board_to_gui, board_to_wind_cable]
        self.all_link_names = ['blue_button', 'red_button', 'slider_start', 'slider_end', 'meter_plug_black', 'meter_plug_red', 'door_knob', 'gui', 'wind_cable']
        ns = '/task_board_detector/'
        self.pose_publishers = []
        for name in self.all_link_names:
            topic = ns + name + '_pose'
            self.pose_publishers.append(rospy.Publisher(topic, geometry_msgs.msg.PoseStamped, queue_size=1))

        self.loop_rate = rospy.Rate(10.0)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.transform_utils = TransformUtils()

    def board_pose_cb(self, msg):
        self.board_poses_queue.append(msg)
        if len(self.board_poses_queue) > self.num_detections_of_board:
            self.board_poses_queue.pop(0)

    def run(self):
        while not rospy.is_shutdown():
            if self.fixed_board_pose is None and len(self.board_poses_queue) < self.num_detections_of_board:
                self.loop_rate.sleep()
                continue
            if self.fixed_board_pose is None:
                self.fixed_board_pose = self.get_median_board_pose()
                self.event_out_pub.publish('e_done')
                self.board_pose_sub.unregister()
                self.board_poses_queue = []
            self.tf_broadcaster.sendTransform((self.fixed_board_pose.pose.position.x,
                                               self.fixed_board_pose.pose.position.y,
                                               self.fixed_board_pose.pose.position.z),
                                               (self.fixed_board_pose.pose.orientation.x,
                                                self.fixed_board_pose.pose.orientation.y,
                                                self.fixed_board_pose.pose.orientation.z,
                                                self.fixed_board_pose.pose.orientation.w),
                                               rospy.Time.now(),
                                               "board_link",
                                               "base_link")
            self.fixed_board_pose.header.stamp = rospy.Time.now()
            self.board_pose_pub.publish(self.fixed_board_pose)

            for link_name, offset, publisher in zip(self.all_link_names, self.all_transforms, self.pose_publishers):
                yaw = tf.transformations.euler_from_quaternion([
                            self.fixed_board_pose.pose.orientation.x,
                            self.fixed_board_pose.pose.orientation.y,
                            self.fixed_board_pose.pose.orientation.z,
                            self.fixed_board_pose.pose.orientation.w])[2]
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, offset[2])
                self.tf_broadcaster.sendTransform((offset[0],
                                                   offset[1],
                                                   0.0),
                                                   (quat[0], quat[1], quat[2], quat[3]),
                                                   rospy.Time.now(),
                                                   link_name + '_link',
                                                   "board_link")
                pose = geometry_msgs.msg.PoseStamped()
                pose.pose.position.x = offset[0]
                pose.pose.position.y = offset[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'board_link'
                pose = self.transform_utils.transformed_pose_with_retries(pose, "base_link")
                publisher.publish(pose)

    def get_median_board_pose(self):
        list_x = [pose.pose.position.x for pose in self.board_poses_queue]
        list_y = [pose.pose.position.y for pose in self.board_poses_queue]
        list_z = [pose.pose.position.z for pose in self.board_poses_queue]
        list_yaw = [tf.transformations.euler_from_quaternion([
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w])[2] for pose in self.board_poses_queue]
        med_x = np.median(list_x)
        med_y = np.median(list_y)
        med_z = np.median(list_z)
        med_yaw = np.median(list_yaw)
        med_pose = geometry_msgs.msg.PoseStamped()
        med_pose.pose.position.x = med_x
        med_pose.pose.position.y = med_y
        med_pose.pose.position.z = med_z
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, med_yaw)
        med_pose.pose.orientation.x = quat[0]
        med_pose.pose.orientation.y = quat[1]
        med_pose.pose.orientation.z = quat[2]
        med_pose.pose.orientation.w = quat[3]
        med_pose.header.frame_id = 'base_link'
        return med_pose

def main():
    rospy.init_node("points_of_interest_publisher")
    poip = PointsOfInterestPublisher()
    poip.run()

if __name__ == "__main__":
    main()
