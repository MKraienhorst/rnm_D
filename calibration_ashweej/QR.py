# -*- coding: utf-8 -*-
"""
Created on Mon Jun 10 17:20:41 2019

@author: ashweej
"""

import numpy as np
import cv2
import glob
import checker
N=checker.obtain_n_mat()
from numpy import genfromtxt
#%%
M=[]
for i in range(6,7):
    data=genfromtxt('Tx_'+str(i)+'.csv', delimiter=',')
    M.append(data)
#%%
R=[]
T=[]
Z=np.zeros((3,3))
E=-np.eye(12)
A=[]
b=[]
Z1=np.zeros((9,1))
for i in range(len(M)):
    R.append(M[i][0:3,0:3])
    T.append(M[i][0:3,3])
    
    
    A00=R[i]*N[i][0,0]
    A01=R[i]*N[i][1,0]
    A02=R[i]*N[i][2,0]
    A03=Z
    A10=R[i]*N[i][0,1]
    A11=R[i]*N[i][1,1]
    A12=R[i]*N[i][2,1]
    A13=Z
    A20=R[i]*N[i][0,2]
    A21=R[i]*N[i][1,2]
    A22=R[i]*N[i][2,2]
    A23=Z
    A30=R[i]*N[i][0,3]
    A31=R[i]*N[i][1,3]
    A32=R[i]*N[i][2,3]
    A33=R[i]
    A_temp=np.bmat([[A00,A01,A02,A03],[A10,A11,A12,A13],[A20,A21,A22,A23],[A30,A31,A32,A33]])
   
    A.append(np.hstack((A_temp,E)))
    Ti=-(T[i].reshape(3,-1))
    b.append(np.vstack((Z1,Ti)))


#%% 
b_final=b[0]
A_final=A[0]
for i in range(len(M)-1):
        b_final=np.vstack((b_final,b[i+1]))
        A_final=np.vstack((A_final,A[i+1]))
    

    
    
#%%

Q=np.linalg.qr(A_final)[0]
R=np.linalg.qr(A_final)[1]
    
#%%



R_inv=np.linalg.inv(R)
R_inv.astype(float)
#%%
Q_trans=np.transpose(Q)
Q_trans.astype(float)
#%%

p1=np.matmul(R_inv,Q_trans)
x=np.matmul(p1,b_final)    

#%%


x1=x[:12,-1]
x1=np.asarray(x1)
x1=x1.reshape(3,4,order='F')
x1=np.vstack((x1,np.array([0,0,0,1])))
N1=np.vstack((N[0],np.array([0,0,0,1])))
xx2=M[0]@x1@np.linalg.inv(N1)















