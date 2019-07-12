#!/usr/bin/env python

import rospy
from kinematics.srv import *
from transform_func import *
from forward import *


def handle_trans(req):
    trans = fwd_kine(req.theta)
    ros_pose = np2ros(trans)
    return TransformServResponse(ros_pose)


def trans_server():
    rospy.init_node('trans_server')
    s = rospy.Service('end_effector_pose', TransformServ, handle_trans)
    print "Transform server is ready."
    rospy.spin()


if __name__ == "__main__":
    trans_server()
