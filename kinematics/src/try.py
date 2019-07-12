#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from math import *
from numpy import linalg
import cmath
import time
from transform_func import *
from kinematics.msg import *



def talker():


                             #global(camera) to head (gth), global to end effector(gtt) and base to end effector(bte)
    gte1 = np.mat([[1,0,0,1000],
                [0,0.7109135,-0.7032794,1000],
                [0,0.7032794,0.7109135,0],
                [0,0,0,1]])

    gth_1 = np.mat([[  1.0000000,  0.0000000,  0.0000000, 1000.0],
                 [0.0000000,  1.0000000,  0.0000000, 0.0],
                 [0.0000000,  0.0000000,  1.0000000, 0.0 ],
                 [0.0, 0.0, 0.0, 1.0]])
    x=linalg.inv(gth_1)*gte1
    gth_2 = np.mat([[  0.9848077, -0.1736482,  0.0000000, 1000.0],
                 [0.1736482,  0.9848077,  0.0000000,0.0],
                 [0.0000000,  0.0000000,  1.0000000,176.32 ],
                 [0.0, 0.0, 0.0, 1.0]])
    bte= np.mat([[ -1.0000000,  0.0000073,  0.0000000,-109.15],
              [ -0.0000073, -1.0000000,  0.0000000,-297.6],
              [  0.0000000,  0.0000000,  1.0000000, 596.46],
              [0.0, 0.0, 0.0, 1.0]])
    head_move= linalg.inv(gth_1)*gth_2
    gte2=gte1*head_move
    y=linalg.inv(gth_2)*gte2
    pub = rospy.Publisher('Transform',matrix, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        z=np.asarray(y)
        x=z.reshape(16)
        rospy.loginfo(x)
        pub.publish(x)
        rate.sleep()




talker()



