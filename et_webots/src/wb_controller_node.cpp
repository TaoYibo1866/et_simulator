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
using sensor_msgs::JointState;
using et_msgs::Detection;

#define POSITION_SENSOR_PERIOD_MS 2
#define CAMERA_PERIOD_MS 10

void cmdCb(Twist::ConstPtr msg, Motor* motor1, Motor* motor2)
{
  motor1->setVelocity(msg->angular.z);
  motor2->setVelocity(msg->angular.y);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wb_controller");
  int publish_clock = 1;
  if (argc == 2)
    publish_clock = atoi(argv[1]);
  ROS_INFO("publish_clock = %d", publish_clock);

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
  encoder1->enable(POSITION_SENSOR_PERIOD_MS);
  encoder2->enable(POSITION_SENSOR_PERIOD_MS);
  Camera* camera = robot->getCamera("camera");
  camera->enable(CAMERA_PERIOD_MS);
  camera->recognitionEnable(CAMERA_PERIOD_MS);

  // ros setup
  ros::NodeHandle nh;
  ros::Publisher clock_pub = nh.advertise<Clock>("/clock", 1);
  ros::Publisher detect_gth_pub = nh.advertise<Detection>("detection_groundtruth", 1);
  ros::Publisher joints_gth_pub = nh.advertise<JointState>("joints_groundtruth", 1);
  ros::Subscriber cmd_sub = nh.subscribe<Twist>("cmd_vel", 1, boost::bind(cmdCb, _1, motor1, motor2), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  while (ros::ok() && robot->step(timestep) != -1)
  {
    double t = robot->getTime();
    Clock clock;
    clock.clock.fromSec(t);
    if (publish_clock)
      clock_pub.publish(clock);
    
    // pub sensor data immediately
    if ((int)(t * 1000) % POSITION_SENSOR_PERIOD_MS == 0)
    {
      JointState joints;
      joints.header.stamp = clock.clock;
      joints.name.push_back("ET3116A/joint1");
      joints.name.push_back("ET3116A/joint2");
      joints.position.push_back(encoder1->getValue());
      joints.position.push_back(encoder2->getValue());
      joints_gth_pub.publish(joints);
    }
    
    if ((int)(t * 1000) % CAMERA_PERIOD_MS == 0)
    {
      Detection detect;
      detect.header.stamp = clock.clock;
      detect.valid = false;
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
      detect_gth_pub.publish(detect);
    }
    
    ros::spinOnce();
  }
  delete robot;
  return 0;
}