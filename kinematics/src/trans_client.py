#!/usr/bin/env python

import sys
import rospy
import numpy as np
from kinematics.srv import *


def trans_client(theta_request):
    rospy.wait_for_service('end_effector_pose')
    try:
        transform_final = rospy.ServiceProxy('end_effector_pose', TransformServ)
        response = transform_final(theta_request)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def usage():
    return "%s theta" % sys.argv[0]


# sys.argv[0] is a whitespace
if __name__ == "__main__":
    theta = np.zeros(6)
    if len(sys.argv) == 7:
        for i in range(6):
            theta[i] = float(sys.argv[i + 1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting pose for given theta %s" % theta
    print "End effector pose is \n %s" % trans_client(theta)
