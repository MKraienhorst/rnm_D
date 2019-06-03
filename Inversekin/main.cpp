#include <iostream>
#include <ros/ros.h>
#include <Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;


double* inverseKinematics(double rx,double ry,double rz,double x,double y,double z)
{
//UR5
const double d1=89.159;
const double a2=-425.0;
const double a3=-392.25;
const double d4=109.15;
const double d5=94.65;
const double d6=82.3;

//UR3
/*const double d1=151.9;
const double a2=-243.65;
const double a3=-213.25;
const double d4=112.35;
const double d5=85.35;
const double d6=81.9; */

Matrix3d r06;
double norm=sqrt((rx*rx)+(ry*ry)+(rz*rz));
double a=rx/norm;
double b=ry/norm;
double c=rz/norm;
//Matrix3d r06;
r06 = AngleAxisd(norm, Vector3d(a,b,c));
cout<<r06<<endl;

//end position
Vector3d p06;
p06<<x,y,z;
//transformation matrix
Matrix <double, 4,4> t06;
t06<< r06,p06,0.0d,0.0d,0.0d,1.0d;
cout<<endl<<t06;
//calculating theta1 (2 configurations)
Matrix <double,4,1> D6;
D6<< 0.0d,0.0d,(-d6),1.00d;
Matrix <double,4,1> uv;
uv<< 0.0d,0.0d,0.0d,1.00d;
Vector4d p05=(t06*D6)-uv;
double xx=p05(0);
double yy=p05(1);
double j=(d4/sqrt((xx*xx)+(yy*yy)));
double theta11=atan2(yy,xx)+acos(j)+(M_PI/2);
double theta12=atan2(yy,xx)-acos(j)+(M_PI/2);


Matrix <double, 4,4> t01_1;
t01_1<< cos(theta11),0.0d,sin(theta11),0.0d,
        sin(theta11),0.0d,-cos(theta11),0.0d,
        0.0d,1.0d,0.0d,d1,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t01_2;
t01_2<< cos(theta12),0.0d,sin(theta12),0.0d,
        sin(theta12),0.0d,-cos(theta12),0.0d,
        0.0d,1.0d,0.0d,d1,
        0.0d,0.0d,0.0d,1.0d;

//calculating theta5

Matrix4d t16_1=t01_1.inverse()*t06;
Matrix4d t16_2=t01_2.inverse()*t06;

double theta51 = acos(((t16_1(2,3))-(d4))/d6);
double theta52 = -theta51;
double theta53 = acos(((t16_2(2,3))-(d4))/d6);
double theta54 = -theta53;

//calculating theta6

Matrix4d t61_1=t16_1.inverse();
Matrix4d t61_2=t16_2.inverse();

double theta61=atan2(-t61_1(1,2)/sin(theta51),t61_1(0,2)/sin(theta51));
double theta62=atan2(-t61_1(1,2)/sin(theta52),t61_1(0,2)/sin(theta52));
double theta63=atan2(-t61_2(1,2)/sin(theta53),t61_2(0,2)/sin(theta53));
double theta64=atan2(-t61_2(1,2)/sin(theta54),t61_1(0,2)/sin(theta54));

//calculating theta3


Matrix <double, 4,4> t45_1;
t45_1<< cos(theta51),0.0d,-sin(theta51),0.0d,
        sin(theta51),0.0d,cos(theta51),0.0d,
        0.0d,-1.0d,0.0d,d5,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t45_2;
t45_2<< cos(theta52),0.0d,-sin(theta52),0.0d,
        sin(theta52),0.0d,cos(theta52),0.0d,
        0.0d,-1.0d,0.0d,d5,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t45_3;
t45_3<< cos(theta53),0.0d,-sin(theta53),0.0d,
        sin(theta53),0.0d,cos(theta53),0.0d,
        0.0d,-1.0d,0.0d,d5,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t45_4;
t45_4<< cos(theta54),0.0d,-sin(theta54),0.0d,
        sin(theta54),0.0d,cos(theta54),0.0d,
        0.0d,-1.0d,0.0d,d5,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t56_1;
t56_1<< cos(theta61),-sin(theta61),0.0d,0.0d,
        sin(theta61),cos(theta61),0.0d,0.0d,
        0.0d,0.0d,1.0d,d6,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t56_2;
t56_2<< cos(theta62),-sin(theta62),0.0d,0.0d,
        sin(theta62),cos(theta62),0.0d,0.0d,
        0.0d,0.0d,1.0d,d6,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t56_3;
t56_3<< cos(theta63),-sin(theta63),0.0d,0.0d,
        sin(theta63),cos(theta63),0.0d,0.0d,
        0.0d,0.0d,1.0d,d6,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t56_4;
t56_4<< cos(theta64),-sin(theta64),0.0d,0.0d,
        sin(theta64),cos(theta64),0.0d,0.0d,
        0.0d,0.0d,1.0d,d6,
        0.0d,0.0d,0.0d,1.0d;


Vector4d D4;
D4<< 0.0d,-d4,0.0d,1.0d;

Matrix <double, 4,4> t14_1;
t14_1=t16_1*t56_1.inverse();
t14_1=t14_1*t45_1.inverse();
Vector4d p13_1=(t14_1*D4)-uv;
double k1 = sqrt((p13_1(0)*p13_1(0))+(p13_1(1)*p13_1(1))+(p13_1(2)*p13_1(2)));
double theta31=acos(((k1*k1)-(a2*a2)-(a3*a3))/(2*a2*a3));
double theta32=-theta31;

Matrix <double, 4,4> t14_2;
t14_2=t16_1*t56_2.inverse();
t14_2=t14_2*t45_2.inverse();
Vector4d p13_2=t14_2*D4;
double k2 = sqrt((p13_2(0)*p13_2(0))+(p13_2(1)*p13_2(1))+(p13_2(2)*p13_2(2)));
double theta33=acos(((k2*k2)-(a2*a2)-(a3*a3))/(2*a2*a3));
double theta34=-theta33;

Matrix <double, 4,4> t14_3;
t14_3=t16_2*t56_3.inverse();
t14_3=t14_3*t45_3.inverse();
Vector4d p13_3=t14_3*D4;
double k3 = sqrt((p13_3(0)*p13_3(0))+(p13_3(1)*p13_3(1))+(p13_3(2)*p13_3(2)));
double theta35=acos(((k3*k3)-(a2*a2)-(a3*a3))/(2*a2*a3));
double theta36=-theta35;

Matrix <double, 4,4> t14_4;
t14_4=t16_2*t56_4.inverse();
t14_4=t14_4*t45_4.inverse();
Vector4d p13_4=t14_4*D4;
double k4 = sqrt((p13_4(0)*p13_4(0))+(p13_4(1)*p13_4(1))+(p13_4(2)*p13_4(2)));
double theta37=acos(((k4*k4)-(a2*a2)-(a3*a3))/(2*a2*a3));
double theta38=-theta37;

//calculating theta2

double theta21=-atan2(p13_1(1),-p13_1(0))+asin(a3*sin(theta31)/k1);
double theta22=-atan2(p13_1(1),-p13_1(0))+asin(a3*sin(theta32)/k1);
double theta23=-atan2(p13_2(1),-p13_2(0))+asin(a3*sin(theta33)/k2);
double theta24=-atan2(p13_2(1),-p13_2(0))+asin(a3*sin(theta34)/k2);
double theta25=-atan2(p13_3(1),-p13_3(0))+asin(a3*sin(theta35)/k3);
double theta26=-atan2(p13_3(1),-p13_3(0))+asin(a3*sin(theta36)/k3);
double theta27=-atan2(p13_4(1),-p13_4(0))+asin(a3*sin(theta37)/k4);
double theta28=-atan2(p13_4(1),-p13_4(0))+asin(a3*sin(theta38)/k4);


//calculating theta4

Matrix <double, 4,4> t23_1;
t23_1<< cos(theta31),-sin(theta31),0.0d,a3,
        sin(theta31),cos(theta31),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t23_2;
t23_2<< cos(theta32),-sin(theta32),0.0d,a3,
        sin(theta32),cos(theta32),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t23_3;
t23_3<< cos(theta33),-sin(theta33),0.0d,a3,
        sin(theta33),cos(theta33),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t23_4;
t23_4<< cos(theta34),-sin(theta34),0.0d,a3,
        sin(theta34),cos(theta34),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t23_5;
t23_5<< cos(theta35),-sin(theta35),0.0d,a3,
        sin(theta35),cos(theta35),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t23_6;
t23_6<< cos(theta36),-sin(theta36),0.0d,a3,
        sin(theta36),cos(theta36),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t23_7;
t23_7<< cos(theta37),-sin(theta37),0.0d,a3,
        sin(theta37),cos(theta37),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t23_8;
t23_8<< cos(theta38),-sin(theta38),0.0d,a3,
        sin(theta38),cos(theta38),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;

Matrix <double, 4,4> t12_1;
t12_1<< cos(theta21),-sin(theta21),0.0d,a2,
        sin(theta21),cos(theta21),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t12_2;
t12_2<< cos(theta22),-sin(theta22),0.0d,a2,
        sin(theta22),cos(theta22),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t12_3;
t12_3<< cos(theta23),-sin(theta23),0.0d,a2,
        sin(theta23),cos(theta23),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t12_4;
t12_4<< cos(theta24),-sin(theta24),0.0d,a2,
        sin(theta24),cos(theta24),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t12_5;
t12_5<< cos(theta25),-sin(theta25),0.0d,a2,
        sin(theta25),cos(theta25),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t12_6;
t12_6<< cos(theta26),-sin(theta26),0.0d,a2,
        sin(theta26),cos(theta26),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t12_7;
t12_7<< cos(theta27),-sin(theta27),0.0d,a2,
        sin(theta27),cos(theta27),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;
Matrix <double, 4,4> t12_8;
t12_8<< cos(theta28),-sin(theta28),0.0d,a2,
        sin(theta28),cos(theta28),0.0d,0.0d,
        0.0d,0.0d,1.0d,0.0d,
        0.0d,0.0d,0.0d,1.0d;

Matrix4d t34_1;
t34_1=t23_1.inverse()*t12_1.inverse();
t34_1=t34_1*t14_1;


Matrix4d t34_2;
t34_2=t23_2.inverse()*t12_2.inverse();
t34_2=t34_2*t14_1;


Matrix4d t34_3;
t34_3=t23_3.inverse()*t12_3.inverse();
t34_3=t34_3*t14_2;


Matrix4d t34_4;
t34_4=t23_4.inverse()*t12_4.inverse();
t34_4=t34_4*t14_2;


Matrix4d t34_5;
t34_5=t23_5.inverse()*t12_5.inverse();
t34_5=t34_5*t14_3;


Matrix4d t34_6;
t34_6=t23_6.inverse()*t12_6.inverse();
t34_6=t34_6*t14_3;


Matrix4d t34_7;
t34_7=t23_7.inverse()*t12_7.inverse();
t34_7=t34_7*t14_4;


Matrix4d t34_8;
t34_8=t23_8.inverse()*t12_8.inverse();
t34_8=t34_8*t14_4;


double theta41=atan2(t34_1(1,0),t34_1(0,0));
double theta42=atan2(t34_2(1,0),t34_2(0,0));
double theta43=atan2(t34_3(1,0),t34_3(0,0));
double theta44=atan2(t34_4(1,0),t34_4(0,0));
double theta45=atan2(t34_5(1,0),t34_5(0,0));
double theta46=atan2(t34_6(1,0),t34_6(0,0));
double theta47=atan2(t34_7(1,0),t34_7(0,0));
double theta48=atan2(t34_8(1,0),t34_8(0,0));

cout<<"Configuration 1"<<endl;
cout<<theta11*180/M_PI<<endl;
cout<<theta21*180/M_PI<<endl;
cout<<theta31*180/M_PI<<endl;
cout<<theta41*180/M_PI<<endl;
cout<<theta51*180/M_PI<<endl;
cout<<theta61*180/M_PI<<endl<<endl;

/*cout<<"Configuration 2"<<endl;
cout<<theta11*180/M_PI<<endl;
cout<<theta22*180/M_PI<<endl;
cout<<theta32*180/M_PI<<endl;
cout<<theta42*180/M_PI<<endl;
cout<<theta51*180/M_PI<<endl;
cout<<theta61*180/M_PI<<endl<<endl;

cout<<"Configuration 3"<<endl;
cout<<theta11*180/M_PI<<endl;
cout<<theta23*180/M_PI<<endl;
cout<<theta33*180/M_PI<<endl;
cout<<theta43*180/M_PI<<endl;
cout<<theta52*180/M_PI<<endl;
cout<<theta62*180/M_PI<<endl<<endl;

cout<<"Configuration 4"<<endl;
cout<<theta11*180/M_PI<<endl;
cout<<theta24*180/M_PI<<endl;
cout<<theta34*180/M_PI<<endl;
cout<<theta44*180/M_PI<<endl;
cout<<theta52*180/M_PI<<endl;
cout<<theta62*180/M_PI<<endl<<endl;

cout<<"Configuration 5"<<endl;
cout<<theta12*180/M_PI<<endl;
cout<<theta25*180/M_PI<<endl;
cout<<theta35*180/M_PI<<endl;
cout<<theta45*180/M_PI<<endl;
cout<<theta53*180/M_PI<<endl;
cout<<theta63*180/M_PI<<endl<<endl;

cout<<"Configuration 6"<<endl;
cout<<theta12*180/M_PI<<endl;
cout<<theta26*180/M_PI<<endl;
cout<<theta36*180/M_PI<<endl;
cout<<theta46*180/M_PI<<endl;
cout<<theta53*180/M_PI<<endl;
cout<<theta63*180/M_PI<<endl<<endl;

cout<<"Configuration 7"<<endl;
cout<<theta12*180/M_PI<<endl;
cout<<theta27*180/M_PI<<endl;
cout<<theta37*180/M_PI<<endl;
cout<<theta47*180/M_PI<<endl;
cout<<theta54*180/M_PI<<endl;
cout<<theta64*180/M_PI<<endl<<endl;

cout<<"Configuration 8"<<endl;
cout<<theta12*180/M_PI<<endl;
cout<<theta28*180/M_PI<<endl;
cout<<theta38*180/M_PI<<endl;
cout<<theta48*180/M_PI<<endl;
cout<<theta54*180/M_PI<<endl;
cout<<theta64*180/M_PI<<endl<<endl;*/
double* g;
static double anglevalues[6]={theta11,theta21,theta31,theta41,theta51,theta61};
g=anglevalues;
return g;


}

int main (int argc,char **argv){

double rx1,ry1,rz1,x1,y1,z1;
cout<<"Enter the value of rx"<<endl;
cin>>rx1;
cout<<"Enter the value of ry"<<endl;
cin>>ry1;
cout<<"Enter the value of rz"<<endl;
cin>>rz1;
cout<<"Enter the value of x"<<endl;
cin>>x1;
cout<<"Enter the value of y"<<endl;
cin>>y1;
cout<<"Enter the value of z"<<endl;
cin>>z1;
ros::init(argc,arg v,"inverse_kinematics_server");
ros::NodeHandle n;



}

double[] ik(double rx1,double ry1,double rz1,double x1,double y1,double z1){
 double* p=inverseKinematics(rx1,ry1,rz1,x1,y1,z1);
 double[] a={p[0],p[1],p[2],p[3],p[4],p[5]};
 return a;

}
