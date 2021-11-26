#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <et_msgs/Detection.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

using namespace webots;
using rosgraph_msgs::Clock;
using geometry_msgs::Twist;
using geometry_msgs::TwistConstPtr;
using sensor_msgs::JointState;
using et_msgs::Detection;


void cmdCb(TwistConstPtr msg, Motor* motor1, Motor* motor2)
{
  motor1->setVelocity(msg->angular.z);
  motor2->setVelocity(msg->angular.y);
}

int main(int argc, char** argv)
{
  // webots setup
  Robot* robot = new Robot;
  int timestep = (int)robot->getBasicTimeStep();
  Motor* motor1 = robot->getMotor("motor1");
  Motor* motor2 = robot->getMotor("motor2");
  motor1->setPosition(INFINITY);
  motor2->setPosition(INFINITY);
  motor1->setVelocity(0);
  motor2->setVelocity(0);
  PositionSensor* encoder1 = robot->getPositionSensor("encoder1");
  PositionSensor* encoder2 = robot->getPositionSensor("encoder2");
  encoder1->enable(timestep);
  encoder2->enable(timestep);
  Camera* camera = robot->getCamera("camera");
  camera->enable(timestep);
  if (camera->hasRecognition())
    camera->recognitionEnable(timestep);

  // ros setup
  ros::init(argc, argv, "wb_controller");
  ros::NodeHandle nh;
  ros::Publisher clock_pub = nh.advertise<Clock>("/clock", 1);
  ros::Publisher detect_gth_pub = nh.advertise<Detection>("detection_groundtruth", 1);
  ros::Publisher joints_gth_pub = nh.advertise<JointState>("joints_groundtruth", 1);
  ros::Subscriber cmd_sub = nh.subscribe<Twist>("cmd_vel", 1, boost::bind(cmdCb, _1, motor1, motor2));

  while (ros::ok() && robot->step(timestep) != -1)
  {
    double t = robot->getTime();
    Clock clock;
    clock.clock.fromSec(t);
    clock_pub.publish(clock);
    
    // pub sensor data immediately
    JointState joints;
    joints.header.stamp = clock.clock;
    joints.name.push_back("ET3116A/joint1");
    joints.name.push_back("ET3116A/joint2");
    joints.position.push_back(encoder1->getValue());
    joints.position.push_back(encoder2->getValue());
    joints_gth_pub.publish(joints);

    Detection detect;
    detect.header.stamp = clock.clock;
    detect.valid = false;
    if (camera->hasRecognition())
    {
      const CameraRecognitionObject* objs = camera->getRecognitionObjects();
      for (int i = 0; i < camera->getRecognitionNumberOfObjects(); i++)
      {
        if (strcmp(objs[i].model, "target") == 0)
        {
          detect.valid = true;
          double x = -objs[i].position[2];
          double y = -objs[i].position[0];
          double z =  objs[i].position[1];
          detect.horz = atan(-y / x);
          detect.vert = atan( z / x);
          detect.dist = sqrt(x*x + y*y + z*z);
        }
      }
    }
    detect_gth_pub.publish(detect);
    
    // update cmd, 0 <= delay < timestep, due to asynchronization
    ros::spinOnce();
  }
  delete robot;
  return 0;
}