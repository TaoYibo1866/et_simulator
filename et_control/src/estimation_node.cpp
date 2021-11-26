#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <et_msgs/Detection.h>
#include <et_msgs/Estimation.h>
#include "KalmanFilter.h"

#define CTRL_PERIOD 0.01

using std::array;
using sensor_msgs::JointState;
using et_msgs::Detection;
using et_msgs::Estimation;

double joint1;
double joint2;

void jointsCb(JointState msg)
{
  joint1 = msg.position[0];
  joint2 = msg.position[1];
}

void detectCb(Detection msg)
{
  static ros::NodeHandle nh;
  static ros::Publisher est_pub = nh.advertise<Estimation>("estimation", 1);
  static KalmanFilter kf;

  if (!kf.isInitialized())
  {
    if (msg.valid)
    {
      kf.initialize(msg.horz, msg.vert, msg.dist, joint1, joint2);
    }
  }
  else
  {
    kf.predict(CTRL_PERIOD);
    if (msg.valid)
    {
      kf.correct1(msg.horz, msg.vert, msg.dist, joint1, joint2, 30e-6, 10);
    }
  }

  Estimation est;
  est.valid = msg.valid;
  if (msg.valid)
  {
    est.Z[0] = msg.horz;
    est.Z[1] = msg.vert;
    est.Z[2] = msg.dist;

    array<double,  3> Zhat = kf.getZhat(joint1, joint2);
    for (int i = 0; i < 3; ++i)
      est.Zhat[i] = Zhat[i];

    array<double,  6> Xhat = kf.getXhat();
    for (int i = 0; i < 6; ++i)
      est.Xhat[i] = Xhat[i];

    array<double, 36> P    = kf.getP();
    for (int i = 0; i < 36; ++i)
      est.P[i] = P[i];
  }
  est_pub.publish(est);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "estimation");
  sensor_msgs::JointStateConstPtr joints_msg = ros::topic::waitForMessage<JointState>("joints_groundtruth");
  joint1 = joints_msg->position[0];
  joint2 = joints_msg->position[1];

  ros::NodeHandle nh;
  ros::Subscriber joints_sub = nh.subscribe<JointState>("joints_groundtruth", 1, jointsCb);
  ros::Subscriber detect_sub = nh.subscribe<Detection>("detection_measured", 1, detectCb);
  ros::spin();
  return 0;
}
