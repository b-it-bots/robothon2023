#!/usr/bin/env python3

# All utilities related to transform we are collecting here to be reused 
# by the different actions

from __future__ import print_function
import tf
import  math

from typing import List

from geometry_msgs.msg import PoseStamped, Quaternion
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
                                      retries  : int = 5,
                                      execute_arm: bool = False,
                                      offset: List[float] = [0., 0., 0.]) -> Union[PoseStamped, None]:
        """ Transform pose with multiple retries

        input reference_pose: The reference pose.
        input target_frame: The name of the taget frame.
        input retries: The number of retries.
        input execute_arm: If true, the pose will be rotated by 180 degrees around the x axis.
        input offset: [r, p, y] offset in deg to be added to the pose if execute_arm is true.

        :return: The updated state.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        transformed_pose = None
        for i in range(0, retries):
            transformed_pose = self.transform_pose(reference_pose, target_frame)
            if transformed_pose:
                break
        
        if execute_arm:
            # rotate around z axis by 90 degrees
            euler = tf.transformations.euler_from_quaternion(
                [transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z,
                transformed_pose.pose.orientation.w]
            )
            q = tf.transformations.quaternion_from_euler(math.pi + offset[0], offset[1], euler[2] + offset[2])
            transformed_pose.pose.orientation = Quaternion(*q)

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
        
    def get_pose_from_link(self, target_link: str, source_link: str):
        '''
        input: target_link: the link to get the pose of
                source_link: the link to get the pose in
        output: PoseStamped\n
        returns the pose of the link in the source link frame
        '''

        listener = tf.TransformListener()

        # Wait for the transform to become available
        listener.waitForTransform(source_link, target_link, rospy.Time(), rospy.Duration(1.0))

        # Get the pose of the base_link with respect to the map
        (trans, rot) = listener.lookupTransform(target_link, source_link, rospy.Time(0))

        # Convert the translation and rotation to a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = source_link
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]

        return pose_msg
    
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
