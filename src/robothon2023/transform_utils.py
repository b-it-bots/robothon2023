#!/usr/bin/env python3

# All utilities related to transform we are collecting here to be reused 
# by the different actions

from __future__ import print_function
import tf

from geometry_msgs.msg import PoseStamped
from typing import Union
import rospy

class TransformUtils(object):

    """Transform Utils convert tf in different frames and also doiong retries"""

    def __init__(self):

        self.listener = tf.TransformListener()
        # how long to wait for transform (in seconds)
        self.wait_for_transform = 0.1

        self.transform_tries = 5

    

    def transformed_pose_with_retries(self, reference_pose: PoseStamped, 
                                      target_frame: str,
                                      retries  : int = 5) -> Union[PoseStamped, None]:
        """ Transform pose with multiple retries

        :return: The updated state.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        transformed_pose = None
        for i in range(0, retries):
            transformed_pose = self.transform_pose(reference_pose, target_frame)
            if transformed_pose:
                break
        return transformed_pose

    def transform_pose(self, reference_pose: PoseStamped, 
                       target_frame: str) -> Union[PoseStamped, None]:
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
