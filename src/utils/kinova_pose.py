from dataclasses import dataclass
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def get_kinovapose_from_list(pose_list):
    return KinovaPose(pose_list[0], pose_list[1], pose_list[2], pose_list[3], pose_list[4], pose_list[5])

def get_kinovapose_from_pose_stamped(pose: PoseStamped):
    '''
    Converts a PoseStamped message to a KinovaPose.

    input: pose (PoseStamped): The PoseStamped message.
    '''

    # convert the quaternion to euler angles
    quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
    )
    euler = euler_from_quaternion(quaternion)

    # convert the euler angles to degrees
    theta_x_deg = math.degrees(euler[0])
    theta_y_deg = math.degrees(euler[1])
    theta_z_deg = math.degrees(euler[2])

    return KinovaPose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, theta_x_deg, theta_y_deg, theta_z_deg)

@dataclass
class KinovaPose:
    """
    A class representing the pose of an object in 3D space.
    
    Attributes:
        x (float): The x coordinate of the object.
        y (float): The y coordinate of the object.
        z (float): The z coordinate of the object.
        theta_x_deg (float): The angle of rotation around the x-axis, in degrees.
        theta_y_deg (float): The angle of rotation around the y-axis, in degrees.
        theta_z_deg (float): The angle of rotation around the z-axis, in degrees.

    Default Values are 0.0 for all attributes.
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    theta_x_deg: float = 0.0
    theta_y_deg: float = 0.0
    theta_z_deg: float = 0.0

    def __str__(self):
        return f"KinovaPose(x={self.x}, y={self.y}, z={self.z}, theta_x={self.theta_x_deg}, theta_y={self.theta_y_deg}, theta_z={self.theta_z_deg})"

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if isinstance(other, KinovaPose):
            return self.x == other.x and self.y == other.y and self.z == other.z and self.theta_x_deg == other.theta_x_deg and self.theta_y_deg == other.theta_y_deg and self.theta_z_deg == other.theta_z_deg
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.x, self.y, self.z, self.theta_x_deg, self.theta_y_deg, self.theta_z_deg))

    def to_list(self):
        return [self.x, self.y, self.z, self.theta_x_deg, self.theta_y_deg, self.theta_z_deg]
    
    def to_pose_stamped(self, frame_id: str="base_link"):
        '''
        Converts the KinovaPose to a PoseStamped message.

        input: frame_id (str): The frame_id of the PoseStamped message.
        default: frame_id="base_link"
        output: pose_stamped (PoseStamped): The PoseStamped message.
        '''

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position.x = self.x
        pose_stamped.pose.position.y = self.y
        pose_stamped.pose.position.z = self.z

        # convert the euler angles to radians
        theta_x_rad = math.radians(self.theta_x_deg)
        theta_y_rad = math.radians(self.theta_y_deg)
        theta_z_rad = math.radians(self.theta_z_deg)

        # convert the euler angles to a quaternion
        quaternion = quaternion_from_euler(theta_x_rad, theta_y_rad, theta_z_rad)

        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        return pose_stamped