#!/usr/bin/env python

import rospy
from kinematics.srv import *
from kinematics.msg import *

def inv_client(pose_req):
    rospy.wait_for_service('angle_desired')
    try:
        angle_desired = rospy.ServiceProxy('angle_desired', InverseServ)
        response = angle_desired(pose_req)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def usage():
    return "%s ur_pose" % sys.argv[0]


if __name__ == "__main__":

    inv_client(pose_req)
#    desired_pose = np.zeros(6)
#    if len(sys.argv) == 7:
#        for i in range(len(desired_pose)):
#            desired_pose[i] = float(sys.argv[i + 1])
#    else:
#        print usage()
#        sys.exit(1)
#    print "Requesting joint states to achieve pose %s" % desired_pose
#    print "The desired joint states are %s" % inv_client(desired_pose)
