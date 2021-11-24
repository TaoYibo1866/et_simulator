#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#define DEG2RAD M_PI / 180.0

using sensor_msgs::Joy;
using geometry_msgs::Twist;

ros::Publisher cmd_pub;

void joyCb(Joy msg)
{
  Twist cmd;
  cmd.angular.z = -1.0 * 5 * DEG2RAD * msg.axes[2];
  cmd.angular.y =  1.0 * 5 * DEG2RAD * msg.axes[3];
  cmd_pub.publish(cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy");
  ros::NodeHandle nh;
  cmd_pub = nh.advertise<Twist>("cmd_vel", 1);
  ros::Subscriber joy_sub = nh.subscribe<Joy>("joy", 1, joyCb);
  ros::spin();
  return 0;
}