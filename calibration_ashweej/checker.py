# -*- coding: utf-8 -*-
"""
Created on Mon Jun 10 14:33:56 2019

@author: ashweej
"""

def obtain_n_mat():
    import numpy as np
    import cv2
    import glob
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)



    objp = np.zeros((8*5,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:5].T.reshape(-1,2)*40
    objpoints = [] # 3d point in real world space
    imgpoints = []# 2d points in image plane.
    rvecs_array=[]
    tvecs_array=[]
    import os
    path='C:/Users/ashweej/Desktop/tuhh notes/RNM/project/images'
    text_files = [f for f in os.listdir(path) if f.endswith('.png')]
    cam_matrix=np.array([[1.0738275801319478e+03, 0, 9.9578287200819273e+02],[0,
       1.0743645678723904e+03, 5.4332225004179918e+02],[0,0,1]])
    dist_coeff=np.array([ 7.3123552029001671e-02, -1.2547422205800898e-01,
       -1.2387100494023395e-03, 1.0099427515276655e-04,
       5.3119684128312354e-02 ])


    #images = glob.glob('*.png')

    for fname in text_files:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8,5),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            #objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            corners2=corners2.reshape(40,2)
            
            #imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (8,5), corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey(1500)
        
            #cv2.imwrite('img'+fname,img)
        
        
        

            cv2.destroyAllWindows()
           # imgpoints_array=np.asarray(imgpoints)
           # objpoints_array=np.asarray(objpoints)
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, cam_matrix, dist_coeff)
            rvecs_array.append(rvecs)
            tvecs_array.append(tvecs)
    #ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    N_matrix=[]
    for i in range(len(text_files)):
        mat=cv2.Rodrigues(rvecs_array[i])[0]
        N_mat=np.hstack((mat,tvecs_array[i]))
        N_matrix.append(N_mat)
    return N_matrix












