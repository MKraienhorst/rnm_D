#!/usr/bin/env python
# -*- coding: utf-8 -*-


from math import *
import tf.transformations as tf
from geometry_msgs.msg import Pose


def ur2ros(ur_pose):
    """Transform pose from UR format to ROS Pose format"""

    # ROS pose
    ros_pose = Pose()

    # Position
    ros_pose.position.x = ur_pose[0]
    ros_pose.position.y = ur_pose[1]
    ros_pose.position.z = ur_pose[2]

    # angle normalized
    angle = sqrt(ur_pose[3] ** 2 + ur_pose[4] ** 2 + ur_pose[5] ** 2)

    orient = [i / angle for i in ur_pose[3:6]]
	#rx ry rz to rotation matrix
    np_T = tf.rotation_matrix(angle, orient)
	#rotation matrix to quaternion
    np_q = tf.quaternion_from_matrix(np_T)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]

    return ros_pose


def ros2np(ros_pose):
    """Transform pose from ROS Pose format to np.array format"""

        # orientation
    np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y,
                                    ros_pose.orientation.z, ros_pose.orientation.w])

    # position
    np_pose[0][3] = ros_pose.position.x
    np_pose[1][3] = ros_pose.position.y
    np_pose[2][3] = ros_pose.position.z

    return np_pose


def np2ros(np_pose):
    """Transform pose from np.array format to ROS Pose format"""

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = np_pose[0, 3]
    ros_pose.position.y = np_pose[1, 3]
    ros_pose.position.z = np_pose[2, 3]

    # ROS orientation
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]

    return ros_pose

