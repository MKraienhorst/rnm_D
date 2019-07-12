#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import *
from numpy import linalg
from transform_func import *
import tf.transformations as tf
import cmath
import time
from kinematics.msg import *
from geometry_msgs.msg import Transform
#def eth_calculation(cth,ctb,bte):
        # cth - camera to head 			(image visualizatiom)
        # ctb - camera to base 			(head eye calibration)
        # bte - base to end effector 		(forward kinematics)

#    eth=linalg.inv(bte)*linalg.inv(ctb)*cth
#    const_eth=eth
#    time.sleep(0.5)


class motionCompensation:
    def __init__(self):
        self.n = 1
        self.c = 0
        self.ctb = np.array([[0.01398894,-0.03011469,0.20371533,500],[-0.00621014,0.0211394,0.02123758,600],[0.04958275,-0.04232279,-0.0213095,1300],[0,0,0,1]])
        rospy.init_node('Motion', anonymous=True)
        rospy.Subscriber("Tcurrent", matrix,self.bte_new_calculation )
        rospy.Subscriber("head_pose", Transform,self.headtracking)
        self.pub = rospy.Publisher('Transform',matrix, queue_size=10)

        rospy.spin()

    def head2np(self,ros_pose):
        """Transform pose from ROS Pose format to np.array format"""

            # orientation
        np_pose = tf.quaternion_matrix([ros_pose.rotation.x, ros_pose.rotation.y,
                                        ros_pose.rotation.z, ros_pose.rotation.w])

        # position
        np_pose[0][3] = ros_pose.translation.x
        np_pose[1][3] = ros_pose.translation.y
        np_pose[2][3] = ros_pose.translation.z

        return np_pose


    def headtracking(self,data):
        self.c=motionCompensation.head2np(self,data)
        print(self.c)

    def bte_new_calculation(self,data):
        #ctb=np.mat([[1 ,0 ,0,100],[0,1,0,200],[0,0,1,300],[0,0,0,1]])
        print('hi')

        cth=self.c

        ne =np.asarray(data.pose)
        nz=ne.reshape((4,4))
        if (self.n==1):
            eth=linalg.inv(nz)*linalg.inv(self.ctb)*cth
            self.const_eth=eth
            self.n=self.n+1
        time.sleep(0.5)
        eth=linalg.inv(nz)*linalg.inv(self.ctb)*cth
#        if self.const_eth.all()!=eth.all():
        bte_new=linalg.inv(self.ctb)*cth*linalg.inv(eth)
        z=np.asarray(bte_new)
        x=z.reshape(16)
        rospy.loginfo(x)
        self.pub.publish(x)

#        else:
#            z=np.asarray(nz)
#            x=z.reshape(16)
#            rospy.loginfo(x)
#            self.pub.publish(x)



if __name__ == "__main__":
     MotionCompensation = motionCompensation()




