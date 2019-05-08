//#include <QCoreApplication>
#include<cstdio>
#include<eigen3/Eigen/Dense>
#include<math.h>
#include<string>
#include<iostream>

using namespace std;
//using namespace Eigen;

void forwardKinematics( float theta[6], Eigen::Matrix<float, 3, 1> &position)
{
  //calculation of the transformationmatrix with matrix multiplication
  //initialize DH parameter
/*
 * //UR3
  const float d1= 0.1519;
  const float a2 = -0.24365;
  const float a3 = -0.21325;
  const float d4 = 0.11235;
  const float d5 = 0.08535;
  const float d6 = 0.0819;
  */////
  // UR5
  const float d1= 0.089159;
  const float a2 = -0.425;
  const float a3 = -0.39225;
  const float d4 = 0.10915;
  const float d5 = 0.09465;
  const float d6 = 0.0823;
  float d[6] = {d1, 0, 0, d4, d5, d6};
  float a[6] = {0, a2, a3, 0, 0, 0};
  float alpha[6] = {M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0};
  //initialize transformation matrix
  Eigen::Matrix<float, 4, 4> T06 = Eigen::Matrix<float, 4, 4>::Identity();
  Eigen::Matrix<float, 4, 4> T =T06;
  Eigen::Matrix<float, 4, 4> T_test = Eigen::Matrix<float, 4, 4>::Zero();
  Eigen::Matrix<float, 4, 4> T06_test = Eigen::Matrix<float, 4, 4>::Identity();
  //calculate sine and cosine, if close to zero make them zero
  float ct, st, ca, sa;
  const float epsilon = 0.0001; //tolerance to zero
  for(int i = 0; i < 6; ++i){
    ct = cos(theta[i]);
    if(ct < epsilon && ct > -epsilon ){
      ct = 0;
    }
    st = sin(theta[i]);
    if(st < epsilon && st > -epsilon ){
      st = 0;
    }
    ca = cos(alpha[i]);
    if(ca < epsilon  && ca > -epsilon ){
      ca = 0;
    }
    sa = sin(alpha[i]);
    if(sa < epsilon  && sa > -epsilon ){
      sa = 0;
    }
    //claculate the transformation matrix
    T_test << ct, -ca*st, sa*st, a[i]*ct, st, ca*ct, -sa*ct, a[i]*st, 0, sa, ca, d[i], 0, 0, 0, 1;
    T06_test = T06_test * T_test;
    T06 = T_test * T06;
  }
  cout << "Matrix multipiclation T06: \n" << T06_test << "\nleft multiplication:\n" << T06 << endl;
  //transformation matrix without matrix multiplication
  //
  float c1, c2, c4, c5, c6, cx, sy, s1, s2, s4, s5, s6;
  c1 = cos(theta[1]);
  c2 = cos(theta[2]);
  c4 = cos(theta[4]);
  c5 = cos(theta[5]);
  c6 = cos(theta[6]);
  s1 = sin(theta[1]);
  s2 = sin(theta[2]);
  s4 = sin(theta[4]);
  s5 = sin(theta[5]);
  s6 = sin(theta[6]);
  cx = cos(theta[2]+theta[3]);
  sy = sin(theta[2]+theta[3]);
  T06(0,0) = c1*(cx*(c4*c5*c6-s4*s6)-sy*(s4*c5*c6+c4*s6))+s1*s5*c6;
  T06(0,1) = c1*(cx*(-c4*c5*s6-s4*c6)-sy*(-s4*c5*c6+c4*c6))-s1*s5*s6;
  T06(0,2) = c1*(cx*(-c4*s5)+sy*s4*s5)+s1*c5;
  T06(0,3) = c1*(cx*(-c4*s5*d6+s4*d5)+sy*(s4*s5*d6+c4*d5))+s1*(c5*d6+d4)+a3*c1*cx+a2*c1*c2;
  T06(1,0) = s1*(cx*(c4*c5*c6-s4*s6)-sy*(s4*c5*c6+c4*s6))-c1*s5*c6;
  T06(1,1) = s1*(cx*(-c4*c5*s6-s4*c6)-sy*(-s4*c5*s6+c4*c6))+c1*s5*s6;
  T06(1,2) = s1*(-cx*c4*s5+sy*s4*c5)-c1*c5;
  T06(1,3) = s1*(cx*(-c4*s5*d6+s4*d5)+sy*(s4*s5*d6+c4*d5))-c1*(c5*d6+d4)+(a3*s1*cx+a2*s1*s2);
  T06(2,0) = sy*(c4*c5*c6-s4*s6)+cx*(s4*c5*c6+c4*s6);
  T06(2,1) = sy*(-c4*c5*s6-s4*c6)+cx*(-s4*c5*s6+c4*c6);
  T06(2,2) = -sy*c4*s5-cx*s4*s5;
  T06(2,3) = sy*(-c4*s5*d6+s4*d5)+cx*(-s4*s5*d6-c4*d5)+a3*sy+a2*s2+d1;
 //las row is already correct as T is identity
  cout << "\n T06 hardcoded:\n" << T06 << endl;
  position << T06(0,3), T06(1,3), T06(2,3);
}


int main(int argc, char *argv[])
{
  /*
  QCoreApplication a(argc, argv);

  return a.exec();*/
  float theta[6] = {M_PI/2,M_PI/2,M_PI/2,M_PI/2,M_PI/2,M_PI/2};
//    float theta[6] = {0,0,0,0,0,0};
  Eigen::Matrix<float, 3, 1> position = Eigen::Matrix<float, 3, 1>::Zero();
  forwardKinematics(theta, position);
  cout << "position is : x/y/z << " << position(0) << "/" << position(1) << "/" << position(2) << endl;
  return 0;
}
