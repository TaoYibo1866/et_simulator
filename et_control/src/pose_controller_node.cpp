#include <ros/ros.h>
#include <et_msgs/Estimation.h>
#include <geometry_msgs/Twist.h>

#define CTRL_PERIOD 0.002 // s

#define DEG2RAD M_PI / 180
#define CLAMP(x, lb, ub) x < lb ? lb : x > ub ? ub : x

using et_msgs::Estimation;
using geometry_msgs::Twist;

class PIController
{
public:
  PIController();
  ~PIController(){};
  double getCtrl(double Kp, double Ki, double e, double dt);
private:
  double prev_e_;
  double prev_u_;
};

PIController::PIController()
{
  prev_e_ = 0;
  prev_u_ = 0;
}

double PIController::getCtrl(double Kp, double Ki, double e, double dt)
{
  double du = CLAMP(Kp * (e - prev_e_) + Ki * e * dt, -INFINITY, INFINITY);
  double u = CLAMP(prev_u_ + du, -INFINITY, INFINITY);
  prev_e_ = e;
  prev_u_ = u;
  return u;
}

void estCb(Estimation msg)
{
  static ros::NodeHandle nh;
  static ros::Publisher cmd_pub = nh.advertise<Twist>("cmd_vel", 1);
  static PIController horz_controller;
  static PIController vert_controller;

  if (msg.valid)
  {
    double horz = msg.Z[0];
    double vert = msg.Z[1];

    Twist cmd;
    cmd.angular.z = horz_controller.getCtrl(20, 120, horz, CTRL_PERIOD);
    cmd.angular.y = vert_controller.getCtrl(20, 120, vert, CTRL_PERIOD);
    cmd_pub.publish(cmd);
  }
  else
  {
    Twist cmd;
    cmd.angular.z = 0;
    cmd.angular.y = 0;
    cmd_pub.publish(cmd);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_controller");
  ros::NodeHandle nh;
  ros::Subscriber est_sub = nh.subscribe<Estimation>("estimation", 1, estCb);
  ros::spin();
  return 0;
}