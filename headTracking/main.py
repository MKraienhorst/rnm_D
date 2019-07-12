#!/usr/bin/env python

#https://github.com/FairchildC/ROS-Robotics-by-Example/blob/master/Chapter9_code/crazyflie_autonomous/scripts/detect_crazyflie.py
import sys
import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy
import tf
import math
from headTracking import *
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Transform, Point, Quaternion
import geometry_msgs.msg

debug = True

class HeadTracker:
    

       # This callback function sets parameters regarding the camera.
       # actually not important
    #def camera_data(self, data):
          # set values on the parameter server
	

#2 call back functions as wait_for_message has a very bad frame rate
    def rgb_callback(self,msg):
	try:
	     #image = = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
             self.newImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	except CvBridgeError as e:
	     print(e)

    def depth_callback(self,msg):
	try:
             depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
	     self.depthNp = np.array(depth_image, dtype=np.float32)
        except CvBridgeError as e:
             print(e)
	
    #def imageProcessStart(self, msg):
    def imageProcessStart(self):

          # convert ROS image to OpenCV image
          try:
             #image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
             #face cascade here
             face_cascade = cv.CascadeClassifier('../haarcascade/haarcascade_frontalface_default.xml') #initialice cascade classifier
             gray = cv.cvtColor(self.newImage, cv.COLOR_BGR2GRAY) #convert img to gray as cascade needs it
             cv.imshow('grayscale',gray)
	     cv.waitKey(1000)
	     cv.destroyAllWindows()
	     faces = face_cascade.detectMultiScale(gray, 2, 1) # detect face (inp: img, scaling factor, number of neighboor pixels)
             for (x,y,w,h) in faces:
                crop = 0.0
                y = int(crop*h+y)
                x = int(crop*w+x)
                h = int(h*(1-crop))
                w = int(w*(1-crop))
                window = (x,y,w,h)
             if 'window' in locals():
                cv.imshow("first head", self.newImage[y:y+h,x:x+w])
                cv.waitKey(1000)
		self.startWindow = window
		self.window, _  = CamShift(window,self.newImage, self.newImage)
		self.startImage = self.newImage
                self.foundFace = True
             else:
                 rospy.loginfo("no head was found")

          except CvBridgeError as e:
             print(e)

    #def imageProcess(self, msg):
    def imageProcess(self):
	#process the image with cam shift algorithm to find the new window	 
	  self.window, _  = CamShift(self.startWindow ,self.startImage, self.newImage)
	  #publish the roi of bgr image
	  xvalues = self.window[:,0]
	  yvalues = self.window[:,1]
	  y = np.amax(yvalues)
	  x = np.amax(xvalues)
	  w = x - np.amin(xvalues)
	  h = y - np.amin(yvalues)
	  cv.rectangle(self.newImage,(x-w,y-h),(x,y),(255,0,0),2)
	  try:
	     self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.newImage, "bgr8"))
          except CvBridgeError as e:
             print(e)

    def centroid(self, points3D):
	    divide = points3D.shape[1]
	    allX = np.sum(points3D[0,:])
	    meanX = allX/divide
	    allY = np.sum(points3D[1,:])
	    meanY = allY/divide
	    allZ = np.sum(points3D[2,:])
	    meanZ = allZ/divide
	    centroidPoint = np.array((meanX,meanY,meanZ))
    	    return centroidPoint

    #def depthProcess(self, msg):
    def depthProcess(self):
          # create numpy depth image using defalut passthrough encoding
	  try:
             depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
          except CvBridgeError as e:
             print(e)
	  self.depthNp = np.array(depth_image, dtype=np.float32)


    def __init__(self):
        # initialize
        rospy.init_node("headTracking")
        self.bridge = CvBridge() #cv bridge to make from ros messages opencv images
        self.publisher = rospy.Publisher('head_pose',Transform, queue_size = 10) #publishs the current position of the centroid
	self.image_pub = rospy.Publisher("image_roi",Image) # image publisher that publishs the roi of head
 	self.transformMessage = Transform() #message send to head_pose publisher
        rospy.wait_for_message('/kinect2/hd/camera_info', CameraInfo) #grab camera info -> just to make sure the camera is grabbing images
	rospy.loginfo("Got Camera Info")
	self.subscribeRGB = rospy.Subscriber('/kinect2/hd/image_color_rect', Image, self.rgb_callback, queue_size=1) #get rgb images all the time
	self.subscribeDepth = rospy.Subscriber('/kinect2/hd/image_depth_rect', Image, self.depth_callback, queue_size=1)# get depth images all the time
        #start the detection with finding the face
        self.foundFace = False
        while self.foundFace == False:
        	#msg = rospy.wait_for_message('kinect2/qhd/image_color_rect', Image)
		rospy.sleep(2)
		HeadTracker.imageProcessStart(self)
	
	if debug == True:
            rospy.loginfo("Found the face")
	#msg = rospy.wait_for_message('kinect2/qhd/image_depth_rect', Image)# tried everything with wait for message to make sure we alway have the right frames -> was way to slow      
        depthROIfiltInd,depthROIfilt = filterDepthPoints(self.window, self.startImage, self.depthNp)
        if debug == True:
            rospy.loginfo("filtered depth points")
	self.pointCloudHead = depthTo3D(depthROIfiltInd,depthROIfilt) #transform depth image to 3D points, first inp (row/col (=y/x)) of depth point; second input depth points
	self.centroidStart = HeadTracker.centroid(self,self.pointCloudHead) #calculate the centroid of the point cloud
	self.centroidLastPos = self.centroidStart #first centeroid use as a starting position
	HeadTracker.logMessage(self, self.centroidStart) #send starting position


    def loop(self):
	try:
		while not rospy.is_shutdown():
			#first subscriber to get rgb image and camshift
			#msg = rospy.wait_for_message('/kinect2/qhd/image_color_rect', Image)
			HeadTracker.imageProcess(self)
			#second subscirber to get depth image
			#msg = rospy.wait_for_message('/kinect2/qhd/image_depth_rect', Image)
			#HeadTracker.depthProcess(self)
	
			#check find the points belonging to the head in depth image
			depthROIfiltInd,depthROIfilt = filterDepthPoints(self.window, self.newImage, self.depthNp)
			#tranform depth points of head to 3D coordinates
			self.pointCloudHead = depthTo3D(depthROIfiltInd,depthROIfilt)#transform depth image to 3D points, first inp (row/col (=y/x)) of depth point; second input depth points
			centroidCurrent = HeadTracker.centroid(self,self.pointCloudHead) #calculate the centroid of the point cloud
			translation = centroidCurrent - self.centroidLastPos #translation from last sent position to new pos
			if np.linalg.norm(translation) > 10: #only send a new position if the total translation from last sent position is bigger than 10 mm 
				self.centroidLastPos = centroidCurrent
			HeadTracker.logMessage(self, self.centroidLastPos) #other wise just send the last position again
			##next lines are not needed anymore, as we do not get a rotation 
			#(xr,yr,zr) = rotationMatrixToEulerAngles(self.transformation[0:3,0:3])
			#rospy.loginfo(str(self.transformation[0:3,0:3]))
			# make message
			#angle = sqrt(self.rotation[0] ** 2 + self.rotation[1] ** 2 + self.rotation[2] ** 2)
			#direction = [j / angle for j in self.rotation[0:3]]
			#np_T = tf.transformations.rotation_matrix(angle, direction)
			#q = tf.transformations.quaternion_from_matrix(np_T)
			#q = tf.transformations.quaternion_from_matrix(self.transformation[0:2,0:2])        
	except KeyboardInterrupt:
		self.subscribeRGB.unregister()
		self.subscribeDepth.unregister()
		print('Interrupt!')
	#rospy.spin()
    def logMessage(self, translation):
			self.transformMessage.translation.x = translation[0]
			self.transformMessage.translation.y = translation[1]
			self.transformMessage.translation.z = translation[2]
			## we do not estimate rotations so always no translation
			#q = tf_conversions.transformations.quaternion_from_euler(xr, yr,zr)
			self.transformMessage.rotation.x = 0# q[1]
			self.transformMessage.rotation.y = 0#q[1]
			self.transformMessage.rotation.z = 0#q[2]
			self.transformMessage.rotation.w = 1#q[3]
			self.publisher.publish(self.transformMessage)
			rospy.loginfo("transformation send")

if __name__ == '__main__':

   # start up the detector node and run until shutdown by interrupt
   headTracker = HeadTracker()
   try:
	#while True:
	headTracker.loop()

   except KeyboardInterrupt:
	print('interrupted!')

   # close all terminal windows when process is shut down
cv.destroyAllWindows()
