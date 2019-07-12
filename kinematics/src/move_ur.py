#!/usr/bin/env python
# -*- coding: utf-8 -*-


from std_msgs.msg import String
from inv_client import *
from geometry_msgs.msg import Transform
import numpy as np


def move_robot(data):

    resp= data.pose

    pub = rospy.Publisher("ur_driver/URScript", String, queue_size=5)

    rate = rospy.Rate(10)
    rospy.loginfo("Robot moving")
    while not rospy.is_shutdown():
        pub.publish("movej([%f, %f, %f, %f, %f, %f], 1.4, 1.05)" % (resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]))
        rospy.loginfo("movej([%f, %f, %f, %f, %f, %f], 1.4, 1.05)" % (resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]))
        rate.sleep()
    

def listener():
    rospy.init_node("ur_move_node", anonymous=True)
    rospy.Subscriber("Angles", matrix , move_robot)
    rospy.spin()
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
