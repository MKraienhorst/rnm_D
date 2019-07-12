#!/usr/bin/env python

#https://github.com/FairchildC/ROS-Robotics-by-Example/blob/master/Chapter9_code/crazyflie_autonomous/scripts/detect_crazyflie.py
import sys
import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy
import tf
from headTracking import *
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import geometry_msgs.msg

debug = True

class HeadTracker:
    

       # This callback function sets parameters regarding the camera.
       # actually not important
    def camera_data(self, data):
          # set values on the parameter server
          rospy.set_param('camera_link', data.header.frame_id)  # kinect2_ir_optical_frame
          rospy.set_param('camera_height', data.height)         # sd height is 424 / qhd height is 540
          rospy.set_param('camera_width', data.width)           # sd width is 512 / qhd width is 960
          # set values for local variables
          self.cam_height = data.height
          self.cam_width = data.width

    def imageProcessStart(self, msg):

          # convert ROS image to OpenCV image
          try:
             image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
             #face cascade here
             face_cascade = cv.CascadeClassifier('../haarcascade/haarcascade_frontalface_default.xml') #initialice cascade classifier
             gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY) #convert img to gray as cascade needs it
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
                cv.imshow("first head", image[y:y+h,x:x+w])
                cv.waitKey(1000)
		self.window = window
                self.newImage = image
                self.foundFace = True
             else:
                 rospy.loginfo("no head was found")

          except CvBridgeError as e:
             print(e)

    def imageProcess(self, msg):

          # convert ROS image to OpenCV image
          try:
             image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
          except CvBridgeError as e:
             print(e)
          rospy.loginfo("Got 1st image of loop")
          # create hsv image of scene
          self.newImage = image
          self.window, _  = CamShift(self.window,self.newImage, startImg)
    
    def depthProcess(self, msg):

          # create OpenCV depth image using defalut passthrough encoding
	  rospy.loginfo("Try bridge depth image")
	  try:
             depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
          except CvBridgeError as e:
             print(e)
	  self.depthNp = np.array(depth_image, dtype=np.float32)
	  #depth_array[np.isnan(self.depthNp)] = 0
          #cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
          #cv.imshow("first head", depth_image)
          #cv.waitKey(1000)
	  rospy.loginfo("Got 1st depth image")
          ##transform depth image to numpy here

    def __init__(self):
        # initialize a node called hw2
        rospy.init_node("headTracking")
        #self.pub_tf = tf.TransformBroadcaster()
        self.bridge = CvBridge()
        ##DEBUG
        #cv2.namedWindow("KinectV2", 1)
        rospy.wait_for_message('/kinect2/qhd/camera_info', CameraInfo)
        # Subscribe to Kinect v2 sd camera_info to get image frame height and width
        rospy.Subscriber('/kinect2/qhd/camera_info', CameraInfo, self.camera_data, queue_size=1)
        rospy.loginfo("Got Camera Info")
        # create a window to display results in
        #cv.NamedWindow("image_view", 1)
	depthNode = None
	imageNode = None
        #start the detection with finding the face
        self.foundFace = False
        while self.foundFace == False:
        	msg = rospy.wait_for_message('kinect2/qhd/image_color_rect', Image)
		HeadTracker.imageProcessStart(self, msg)
	
	if debug == True:
            rospy.loginfo("Found the face")
        self.window, _  = CamShift(self.window,self.newImage, self.newImage)
	msg = rospy.wait_for_message('kinect2/qhd/image_depth_rect', Image)        
        HeadTracker.depthProcess(self,msg)
	if debug == True:
            rospy.loginfo("Camshift worked")


        depthROIfiltInd,depthROIfilt = filterDepthPoints(self.window, self.newImage, self.depthNp)
        if debug == True:
            rospy.loginfo("filtered depth points")
        self.pointCloudHead = depthTo3D(depthROIfiltInd,depthROIfilt)
        self.o3dPointCloudStart = makeO3dPoints(self.pointCloudHead)
        if debug == True:
            rospy.loginfo("Got Head Pointcloud")

    def loop(self):
        #first subscriber to get rgb image and camshift
        msg = message_filters.Subscriber('/kinect2/qhd/image_color_rect', Image)
	imageProcess(msg)
        #second subscirber to get depth image
        msg = rospy.wait_for_message('/kinect2/qhd/image_depth_rect', Image)
	depthProcess(msg)
	
        #check find the points belonging to the head in depth image
        depthROIfiltInd,depthROIfilt = filterDepthPoints(self.window, self.newImage, self.depthNp)
        #tranform depth points of head to 3D coordinates
        self.pointCloudHead = depthTo3D(depthROIfiltInd,depthROIfilt)
        #tranform numpy array to open3d point cloud
        self.o3dPointCloud = makeO3dPoints(self.pointCloudHead)
        #start the ICP
	voxelSize = 5
        threshold = 1.5*voxelSize
        trans_init = np.asarray([[1., 0., 0., 0.],[0., 1., 0., 0.],
                                 [0., 0., 1., 0.001], [0.0, 0.0, 0.0, 1.]])
        pose_mat = getHeadPose(depthROIfiltInd, pointCloudHead)
        self.transformation = reg_p2pl.transformation
	rospy.loginfo("Got transformation")
        tfbr = tf.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        (xr,yr,zr) = rotationMatrixToEulerAngles(self.transformation[0:3,0:3])
        t.transform.translation.x = self.tranformation[0,3]
        t.transform.translation.y = self.tranformation[1,3]
        t.transform.translation.z = self.tranformation[2,3]
        q = tf_conversions.transformations.quaternion_from_euler(xr, yr,zr)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tfbr.sendTransform(t)
	
	self.o3dPointCloud = self.o3dPointCloudStart
	rospy.spin()

if __name__ == '__main__':

   # start up the detector node and run until shutdown by interrupt
   headTracker = HeadTracker()
   try:
      headTracker.loop()

   except rospy.ROSInterruptException:
      rospy.loginfo("Detector node terminated.")

   # close all terminal windows when process is shut down
cv2.destroyAllWindows()
