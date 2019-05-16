#include "ros/ros.h"
#include "geometry_msgs/Point.h"

void chatterCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  ROS_INFO("I heard: [%lf %lf %lf]", msg->x,msg->y,msg->z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DListener");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
