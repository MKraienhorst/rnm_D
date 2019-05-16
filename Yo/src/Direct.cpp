#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include<iostream>
#include<math.h>
using namespace std;

double* directkinematics(double a,double b,double c,double d,double e,double f){
double arg[6]={(M_PI/180)*a,M_PI/180*b,M_PI/180*c,M_PI/180*d,M_PI/180*e,M_PI/180*f};
double t1=arg[0];
double t2=arg[1];
double t3=arg[2];
double t4=arg[3];
double t5=arg[4];
double t6=arg[5];
//const double d1=0.1519;
//const double a2=-0.24365;
//const double a3=-0.21325;
//const double d4=0.11235;
//const double d5=0.08535;
//const double d6=0.0819;
const double d1=0.089159;
const double a2=-0.425;
const double a3=-0.39225;
const double d4=0.10915;
const double d5=0.09465;
const double d6=0.0823;
static double matrixentry[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double* g;
g=matrixentry;
matrixentry[0]=(cos(t6)*(sin(t1)*sin(t5)+cos(t2+t3+t4)*cos(t1)*cos(t6))-sin(t2+t3+t4)*cos(t1)*sin(t6));
matrixentry[1]=(-sin(t6)*(sin(t1)*sin(t5)+cos(t2+t3+t4)*cos(t1)*cos(t5))-sin(t2+t3+t4)*cos(t1)*cos(t6));
matrixentry[2]=(cos(t5)*sin(t1)-cos(t2+t3+t4)*cos(t1)*sin(t5));
matrixentry[3]=(d6*(cos(t5)*sin(t1)-cos(t2+t3+t4)*cos(t1)*sin(t5))+d4*sin(t1)+d5*sin(t2+t3+t4)*cos(t1)+cos(t1)*(a3*cos(t2+t3)+a2*cos(t2)));
matrixentry[4]=(-cos(t6)*(cos(t1)*sin(t5)-cos(t2+t3+t4)*cos(t5)*sin(t1))-sin(t2+t3+t4)*sin(t1)*sin(t6));
matrixentry[5]=(sin(t6)*(cos(t1)*sin(t5)-cos(t2+t3+t4)*cos(t5)*sin(t1))-sin(t2+t3+t4)*cos(t6)*sin(t1));
matrixentry[6]=(-cos(t1)*cos(t5)-cos(t2+t3+t4)*sin(t1)*sin(t5));
matrixentry[7]=(sin(t1)*(a3*cos(t2+t3)+a2*cos(t2))-d4*cos(t1)-d6*(cos(t1)*cos(t5)+cos(t2+t3+t4)*sin(t1)*sin(t5))+d5*sin(t2+t3+t4)*sin(t1));
matrixentry[8]=(cos(t2+t3+t4)*sin(t6)+sin(t2+t3+t4)*cos(t5)*cos(t6));
matrixentry[9]=(cos(t2+t3+t4)*cos(t6)-sin(t2+t3+t4)*cos(t5)*sin(t6));
matrixentry[10]=(-sin(t2+t3+t4)*sin(t5));
matrixentry[11]=(d1-d5*cos(t2+t3+t4)+a3*sin(t2+t3)+a2*sin(t2)-d6*sin(t2+t3+t4)*sin(t5));
matrixentry[12]=0;
matrixentry[13]=0;
matrixentry[14]=0;
matrixentry[15]=1;

return g;

}

int main(int argc, char **argv)
{
    double a,b,c,d,e,f;
    cout<<"Enter theta1 :"<<endl;
    cin>>a;
    cout<<"Enter theta2 :"<<endl;
    cin>>b;
    cout<<"Enter theta3 :"<<endl;
    cin>>c;
    cout<<"Enter theta4 :"<<endl;
    cin>>d;
    cout<<"Enter theta5 :"<<endl;
    cin>>e;
    cout<<"Enter theta6 :"<<endl;
    cin>>f;

    double* p=directkinematics(a,b,c,d,e,f);
    for(int i=0;i<=15;i++){
        cout<<"a"<<i<<": "<<p[i]<<endl;
}
cout<<endl;
double x=p[3];
double y=p[7];
double z=p[11];
int count=0;
  ros::init(argc, argv, "Direct");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Point>("chatter", 1000);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {

    geometry_msgs::Point msg;

    msg.x =x;
    msg.y=y;
    msg.z=z;

ROS_INFO("%d  X:%lf  Y:%lf  Z:%lf",count,x,y,z);
    chatter_pub.publish(msg);
    ros::spinOnce();


    loop_rate.sleep();

      count++;
  }

  return 0;
}
