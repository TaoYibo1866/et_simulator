#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_controller");
  ros::spin();
  return 0;
}