#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <et_msgs/Detection.h>
#include <et_msgs/Estimation.h>
#include "KalmanFilter.h"

#define PREDICT_PERIOD 0.002  // s
#define CORRECT_PERIOD 0.01   // s

using std::array;
using sensor_msgs::JointState;
using std_msgs::Int8;
using et_msgs::Detection;
using et_msgs::Estimation;

JointState joint_state;
Detection detect;

enum CALLBACK_ID
{
  JOINT_STATE,
  DETECTION
};

void jointsCb(JointState msg)
{
  static ros::NodeHandle nh;
  static ros::Publisher joint_pub = nh.advertise<Int8>("filter_event", 1);
  joint_state = msg;

  Int8 event;
  event.data = JOINT_STATE;
  joint_pub.publish(event);
}

void detectCb(Detection msg)
{
  static ros::NodeHandle nh;
  static ros::Publisher detect_pub = nh.advertise<Int8>("filter_event", 1);
  detect = msg;
  Int8 event;
  event.data = DETECTION;
  detect_pub.publish(event);
}

void filterCb(Int8 msg)
{
  static ros::NodeHandle nh;
  static ros::Publisher est_pub = nh.advertise<Estimation>("estimation", 1);
  static KalmanFilter kf;

  if (msg.data == JOINT_STATE)
  {
    if (kf.isInitialized())
    {
      kf.predict(PREDICT_PERIOD);
    }

    Estimation est;
    est.valid = detect.valid;
    if (detect.valid)
    {
      est.Z[0] = detect.horz;
      est.Z[1] = detect.vert;
      est.Z[2] = detect.dist;

      array<double,  3> Zhat = kf.getZhat(joint_state.position[0], joint_state.position[1]);
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

  if (msg.data == DETECTION)
  {
    if (!kf.isInitialized())
    {
      if (detect.valid)
      {
        kf.initialize(detect.horz, detect.vert, detect.dist, joint_state.position[0], joint_state.position[1]);
      }
    }
    else
    {
      if (detect.valid)
      {
        kf.correct1(detect.horz, detect.vert, detect.dist, joint_state.position[0], joint_state.position[1], 30e-6, 10);
      }
    }
  }

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "estimation");

  ros::NodeHandle nh;
  ros::Subscriber joints_sub = nh.subscribe<JointState>("joints_groundtruth", 1, jointsCb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber detect_sub = nh.subscribe<Detection>("detection_measured", 1, detectCb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber filter_sub = nh.subscribe<Int8>("filter_event", 1, filterCb, ros::TransportHints().tcpNoDelay());
  ros::spin();
  return 0;
}
