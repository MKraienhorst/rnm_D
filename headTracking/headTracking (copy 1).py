
import sys
sys.path.append("/usr/local/lib/python2.7/dist-packages/open3d/")
import os
import cv2 as cv
import numpy as np
from scipy import stats
import open3d as o3d
import math
##for iai's convert_calib_pose_to_urdf_format.py
#import argparse
#import importlib
#import conCalPose

## intialize global variables
#parameters from color yaml, projection for what is it needed?!

hdCount = 2 #make to 1 for hd images, =2 for qhd
fx = 1.0738275801319478e+03/hdCount
fy = 1.0743645678723904e+03/hdCount
cx = 9.9578287200819273e+02/hdCount
cy = 5.4332225004179918e+02/hdCount
camera = np.array([ [fx, 0., cx],[ 0.,fy, cy],[ 0., 0., 1 ]])
distortion = np.array([ 7.3123552029001671e-02, -1.2547422205800898e-01, -1.2387100494023395e-03, 1.0099427515276655e-04,5.3119684128312354e-02 ])
#DEBUG
showHeadPoints = 0
#DEBUG END

#parameters from depth yaml
depthShift      = -1.8578675638819288e+01        
#puts all jpg in the folder into a list 
#input: folder which should be checked for images
#returns the list with path to the images
def getImgsInFolder(path):
    images = []
    for file in os.listdir(path):
        if file.endswith(".jpg") or file.endswith(".png") or file.endswith(".pgm"):
            images.append(path + '/' + file)
    return images

def getNumpysInFolder(path):
    numpiesLoad = []
    for file in os.listdir(path):
        if file.endswith(".npy"):
            numpiesLoad.append(path + '/' + file)
    return numpiesLoad
#applies opencv's haarcascade -> for face onto image
#input: image
#output: the roi where the face is found + the image with the roi marked with a box
#
#example of which classifiers are the best https://www.learnopencv.com/face-detection-opencv-dlib-and-deep-learning-c-python/
def faceCascade(img):
    face_cascade = cv.CascadeClassifier('../haarcascade/haarcascade_frontalface_default.xml') #initialice cascade classifier
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) #convert img to gray as cascade needs it
    faces = face_cascade.detectMultiScale(gray, 2, 1) # detect face (inp: img, scaling factor, number of neighboor pixels)
    for (x,y,w,h) in faces:
        crop = 0.0
        y = int(crop*h+y)
        x = int(crop*w+x)
        h = int(h*(1-crop))
        w = int(w*(1-crop))
        #cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        window = (x,y,w,h)
        
        #cv.drawContours(img,(x,y),0,(0,255,0),2)
        #roi_gray = gray[y:y+h, x:x+w]
        #roi_color = img[y:y+h, x:x+w]
    if 'window' in locals():
        return window, img
    else:
        print('no face was detected')

#find the previous segmented face again in the next frame using opencv's CamShift algorithm
#input: box around the face in previous img, previous img, new img
#output: box around head in new img, new img
def CamShift(track_window,currentImg, nextImg):
    hueUpLim = 130.
    #track_window = (x,y,w,h) #use the found face for start of the camshift algorithm
    roi = currentImg[track_window[1]:track_window[1]+track_window[3], track_window[0]:track_window[0]+track_window[2]] #define the region of interest to the found face
    hsv_roi =  cv.cvtColor(roi, cv.COLOR_BGR2HSV) # convert to hsv
    mask = cv.inRange(hsv_roi, np.array((hueUpLim, 60.,32.)), np.array((180.,255.,255.))) #segment the image -> can be enhanced with only segmenting pink and red colors
    roi_hist = cv.calcHist([hsv_roi],[0],mask,[180],[0,180]) # calculate the hsv histogram of the found face
    cv.normalize(roi_hist,roi_hist,0,255,cv.NORM_MINMAX)#normalize the roi
    # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    term_crit = ( cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1 )#stopping criteria of the camshift algorithm -> not optimized, still starting condition
    #cv.imshow('1st image with box', currentImg)
    hsv = cv.cvtColor(nextImg, cv.COLOR_BGR2HSV)#convert new image to hsv
    dst = cv.calcBackProject([hsv],[0],roi_hist,[0,180],1)#use backpropagation of the histogram of the face found in the last image on the new image
    # apply meanshift to get the new location
    ret, track_window = cv.CamShift(dst, track_window, term_crit)# cam shift algorithm to track the window
    # Draw it on image
    pts = cv.boxPoints(ret)#convert output of camshift to draw the surrounding box of the face, found in the new image
    pts = np.int0(pts)
    ##to check the found bounding box without tilt
    #xvalues = pts[:,0]
    #yvalues = pts[:,1]
    #y = np.amax(yvalues)
    #x = np.amax(xvalues)
    #w = x- np.amin(xvalues)
    #h = y - np.amin(yvalues)
    #cv.rectangle(nextImg,(x-w,y-h),(x,y),(255,0,0),2)
    #cv.drawContours(nextImg,[pts], 0,(0,0,255),2)
    #cv.imshow('nextImg',nextImg)
    #cv.imshow('dst',dst)
    #cv.waitKey(1000)
    return pts, nextImg
    #undistort color image -> can be improved to only undistorting points in roi_color
    #map the depth image to 3D point cloud

# depthROIfiltInd: 2xN array with the rows and columns of the ROI
# depthROIfilt:    1xN array with the corresponding depth of the ROI 
def depthTo3D(depthROIfiltInd,depthROIfilt):
    numPoints = depthROIfilt.size
    pointCloudHead = np.zeros((3,numPoints))
    for i in range(numPoints):
        pointCloudHead[0,i] = (depthROIfiltInd[1,i]-cx)*depthROIfilt[i]/fx
        pointCloudHead[1,i] = -(depthROIfiltInd[0,i]-cy)*depthROIfilt[i]/fy #flip the y axis
        pointCloudHead[2,i] = depthROIfilt[i]
    return pointCloudHead


def filterDepthPoints(window,bgrImg, depthNP):
    #4 corners of the rectangle
    maxCol = np.amax(window[:,0])
    minCol = np.amin(window[:,0])
    maxRow = np.amax(window[:,1])
    minRow = np.amin(window[:,1])
    ##take the ROI of depth image and make 3D points out of it
    #depthROI = depthNP[minCol:maxCol,minRow:maxRow]
    depthROI = depthNP[minRow:maxRow,minCol:maxCol]
    ##show only the ROI of RGB image
    #imgROI = bgrImg[minRow:maxRow,minCol:maxCol]
    #cv.imshow('ROI', imgROI)
    #cv.waitKey(1000)

    ##filter the ROI of depth image with calculating the mode of all depth that are not zero
    #then filter around +-400mm around the mode and take that as the head 
    depthNoZero = depthROI[np.nonzero(depthROI)]
    depthRound = np.around(depthNoZero)
    modeDis = stats.mode(depthRound,axis=None)
    diffFromMode = 70
    depthROIfiltInd = np.where((depthROI < modeDis[0]+diffFromMode)&(depthROI > modeDis[0]-diffFromMode)) #row and col of ROI of points that belong to the head
    depthROIfilt = depthROI[depthROIfiltInd] #depth of the points that belong to the head
    depthROIfiltIndex = np.zeros((2,depthROIfiltInd[0].size),dtype=int)
    depthROIfiltIndex[0,:] = depthROIfiltInd[0]
    depthROIfiltIndex[1,:] = depthROIfiltInd[1]
    depthROIfilt = depthROI[depthROIfiltIndex[0,:],depthROIfiltIndex[1,:]] #depth of the points that belong to the head
    #plot found points in the ROI
    if showHeadPoints == 1:
        imgROI = bgrImg[minRow:maxRow,minCol:maxCol]
        for i in range(depthROIfilt.size):
            cv.circle(imgROI,(depthROIfiltInd[1][i],depthROIfiltInd[0][i]), 1, (0,0,255), -1)
    #cv.imshow('markedROI', imgROI)
    #cv.waitKey(0)
    #plt.hist(depthROIfilt,bins = bin)
    #plt.show()
    #for u,v in len(depthROI):
    #    if ((depthROI[u,v] > modeDistance + disToMode) && ((depthROI[u,v] > modeDistance - disToMode))

    #transform 
    #is the first rows or columns
    depthROIfiltIndex[0,:] = depthROIfiltIndex[0,:]+minRow # transform rows of ROI to rows of whole image
    depthROIfiltIndex[1,:] = depthROIfiltIndex[1,:]+minCol # transform cols of ROI to cols of whole image
    return depthROIfiltIndex, depthROIfilt

#by https://github.com/lincolnhard/head-pose-estimation/blob/master/video_test_shape.py
def getHeadPose(depthROIfiltInd, pointCloudHead):
    imagePoints = np.zeros((depthROIfiltInd.shape[1],1,2),dtype=np.float32)
    imagePoints[:,0,0] = depthROIfiltInd[1,:]
    imagePoints[:,0,1] = depthROIfiltInd[0,:]
    objectPoints = np.zeros((depthROIfiltInd[0].size,1,3),dtype=np.float32)
    objectPoints[:,0,0] = pointCloudHead[0,:]
    objectPoints[:,0,1] = pointCloudHead[1,:]
    objectPoints[:,0,2] = pointCloudHead[2,:]
    pointCloudHead = np.float64(pointCloudHead)
    #imagePoints = np.float32(imagePoints)
    flag = cv.SOLVEPNP_ITERATIVE
    rvec = np.zeros((3,1))
    #tvec = np.array([0, 0, 500])
    rotation  = np.array([[1, 0, 0],[0, -1, 0],[0, 0, 1]])
    cameranew= np.matmul(camera, rotation)
    tvec = np.zeros(3)
    dist_coefs = np.zeros(4,'float32')
    _, rotation_vec, translation_vec = cv.solvePnP(objectPoints, imagePoints, camera, distortion, flags=flag)
    _, rotation_vec1, translation_vec1 = cv.solvePnP(objectPoints, imagePoints, cameranew, None, flags=flag)
    #translation_vec = -100*translation_vec
    #(success, rotation_vec, translation_vec) = cv.solvePnPRansac(objectPoints, imagePoints, camera, dist_coeffs, flags=cv.SOLVEPNP_P3P)
    # calc position
    rotation_mat, _ = cv.Rodrigues(rotation_vec)
    pose_mat = cv.hconcat((rotation_mat, translation_vec))
    return pose_mat

#http://www.open3d.org/docs/release/tutorial/Advanced/global_registration.html
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    o3d.geometry.estimate_normals(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size,source, target):
    print(":: Load two point clouds and disturb initial pose.")
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.001], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    #draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size,result_ransac):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(startHead)
    downSource = o3d.geometry.voxel_down_sample(source, voxel_size=5)
    o3d.geometry.estimate_normals(downSource, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1,max_nn=30))

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])



def makeO3dPoints(headPoints):
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(headPoints)
    downSource = o3d.geometry.voxel_down_sample(source, voxel_size=5)
    o3d.geometry.estimate_normals(downSource, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1,max_nn=30))
    return downSource

#test function to check if haarcascade + camshift algorithm are an option for pose estimation
def testHeadTracking(folderPath):
    #load starting images and numpy
    startImg = cv.imread('../20190626/color/z10mm_color.pgm')
    startDepthImg = cv.imread('../20190626/depth/z10mm_depth.pgm')
    startDepthNP = np.load('../20190626/depth/z10mm_depth.npy')
    #startImg = cv.imread('../distorted/color/start_color.pgm')
    #startDepthImg = cv.imread('../distorted/depth/start_depth.pgm')
    #startDepthNP = np.load('../distorted/depth/start_depth.npy')
    #load all color images, depth images and depth np into lists
    depthPath = folderPath + '/depth'
    colorPath = folderPath + '/color'
    rgbImages = getImgsInFolder(colorPath) #get all images in the folder
    depthImages = getImgsInFolder(depthPath)
    numpyLoad = getNumpysInFolder(depthPath)
    
    ##undistort image
    #startImgUndi = cv.undistort(startImg, camera, distortion)
    
    startWindow, startImg = faceCascade(startImg) #  segment with opencv haarcascade the face in the first image
    #startWindow, startImgUndi = faceCascade(startImgUndi) #  segment with opencv haarcascade the face in the first image

    #point cloud of the face in first image
    #landMarks, newWindow = landMarksInROI(startImg,startWindow,i)
    #do cam shift again to have exact same results
    window, startImg = CamShift(startWindow,startImg, startImg)
    depthROIfiltInd,depthROIfilt = filterDepthPoints(window, startImg,startDepthNP)
    #window, startImgUndi = CamShift(startWindow,startImg, startImgUndi)
    #depthROIfiltInd,depthROIfilt = filterDepthPoints(window, startImgUndi, startDepthImg,startDepthNP)
    pointCloudHead = depthTo3D(depthROIfiltInd,depthROIfilt)
    startHead = np.transpose(pointCloudHead)
    poseMat = getHeadPose(depthROIfiltInd, pointCloudHead)
    
    #print("translation: " + str(poseMat[:,3]))
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(startHead)
    downSource = o3d.geometry.voxel_down_sample(source, voxel_size=5)
    o3d.geometry.estimate_normals(downSource, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1,max_nn=30))
    
    for i in range(len(rgbImages)):
        #now use camshift
        nextImg = cv.imread(rgbImages[i])
        depthImg = cv.imread(depthImages[i])
        depthNP = np.load(numpyLoad[i])
        #nextImg = startImg
        #depthImg = startDepthImg
        #depthNP = startDepthNP
        depthROIfilt = []
        depthROIfiltInd = []
        nextPoseMat = []
        relative = []
        pointCloudHead = []
        window = []
        diff = []
        window, nextImg = CamShift(startWindow,startImg, nextImg)
        depthROIfiltInd,depthROIfilt = filterDepthPoints(window, nextImg,depthNP)
        pointCloudHead = depthTo3D(depthROIfiltInd,depthROIfilt)
        nextPoseMat = getHeadPose(depthROIfiltInd, pointCloudHead)
        print('firstPos:' + str(poseMat[:,3]))
        #print('secondPos:' + str(nextPoseMat[:,3]))
        ##relative = poseMat.nor-nextPoseMat
        
        ##print(relative) 
        #diff = np.linalg.norm(poseMat)-np.linalg.norm(nextPoseMat)
        #print("moved " + str(diff) + "mm")
        #poseMat = nextPoseMat
        print("for image: ")
        print(depthImages[i])
        #use ICP for relative pose estimation
        voxelSize = 5
        target = o3d.geometry.PointCloud()
        nextHead = np.transpose(pointCloudHead)
        target.points = o3d.utility.Vector3dVector(nextHead)
        downTarget = o3d.geometry.voxel_down_sample(target, voxel_size=voxelSize)
        o3d.geometry.estimate_normals(downTarget, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10,max_nn=30))
        threshold = 1.5*voxelSize
        trans_init = np.asarray([[1., 0., 0., 0.],[0., 1., 0., 0.],
                                 [0., 0., 1., 0.001], [0.0, 0.0, 0.0, 1.]])
        #evaluation = o3d.registration.evaluate_registration(downSource, downTarget,threshold, trans_init)
        reg_p2pl = o3d.registration.registration_icp(downSource, downTarget, threshold, trans_init, o3d.registration.TransformationEstimationPointToPlane())
        print("Transformation is:")
        print(reg_p2pl.transformation)
        print("moved: ")
        (x,y,z) = rotationMatrixToEulerAngles(reg_p2pl.transformation[0:3,0:3])
        o3dMovement1 = np.linalg.norm(reg_p2pl.transformation[0:3,3])
        print(str(o3dMovement1) + "mm")
        print("rotated" + str(x) + " " + str(y) + " "+ str(z))

        #voxel_size = 4  # means 5mm for the dataset
        #source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size,source,target)

        #result_ransac = execute_global_registration(source_down, target_down,
        #                                            source_fpfh, target_fpfh,
        #                                            voxel_size)
        #o3dMovementRan = np.linalg.norm(result_ransac.transformation[0:3,3])
        #print("ransac result: " + str(result_ransac.transformation))
        #print("ransac moved " + str(o3dMovementRan))
        #result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size, result_ransac)
        #print(result_icp.transformation)
        #o3dMovement = np.linalg.norm(result_icp.transformation[0:3,3])
        #print("moved " + str(o3dMovement) + "mm")
       
       #draw_registration_result(source, target, result_icp.transformation)
    debug = 1

if __name__== "__main__":
    newPose = '../20190626'
    testHeadTracking(newPose)    
    debug = 1