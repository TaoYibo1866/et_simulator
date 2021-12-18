#include <ros/ros.h>
#include <et_msgs/Detection.h>
#include <random>
#include <deque>

#define FREQ_DIV 2
#define CACHE_SIZE 6
#define RESOLUTION 30e-6
#define HORZ_VERT_STDDEV 20e-6
#define DIST_STDDEV 10

using et_msgs::Detection;

double quantizer(double val, double resolution)
{
  if (resolution == 0)
    return val;
  return floor(val / resolution) * resolution;
}

void detectCb(Detection msg)
{
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<Detection>("detection_measured", 1);
  static std::default_random_engine gen;
  static std::normal_distribution<double> noise_horz(0, HORZ_VERT_STDDEV);
  static std::normal_distribution<double> noise_vert(0, HORZ_VERT_STDDEV);
  static std::normal_distribution<double> noise_dist(0, DIST_STDDEV);
  static std::deque<Detection> cache;
  static int cnt = 0;

  cache.push_back(msg);
  while (cache.size() > CACHE_SIZE)
    cache.pop_front();

  cnt = cnt % FREQ_DIV;
  if (cnt == 0)
  {
    Detection measured;
    measured.header = cache.front().header;
    measured.valid = cache.front().valid;
    if (cache.front().valid)
    {
      measured.horz = quantizer(cache.front().horz + noise_horz(gen), RESOLUTION);
      measured.vert = quantizer(cache.front().vert + noise_vert(gen), RESOLUTION);
      measured.dist = cache.front().dist + noise_dist(gen);  
    }
    pub.publish(measured);
  }
  ++cnt;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_character");
  ros::NodeHandle nh;
  ros::Subscriber detect_sub = nh.subscribe<Detection>("detection_groundtruth", 1, detectCb);
  ros::spin();
  return 0;
}