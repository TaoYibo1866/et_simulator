#include <ros/ros.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <et_msgs/Detection.h>
#include <random>

#define FREQ_FACTOR 1
#define CACHE_SIZE 2
#define RESOLUTION 30e-6
#define HORZ_VERT_STDDEV 20e-6
#define DIST_STDDEV 10

using et_msgs::Detection;
using et_msgs::DetectionConstPtr;

std::default_random_engine gen;
std::normal_distribution<double> noise_horz;
std::normal_distribution<double> noise_vert;
std::normal_distribution<double> noise_dist;

double quantizer(double val, double resolution)
{
  if (resolution == 0)
    return val;
  return floor(val / resolution) * resolution;
}

void cacheCb(DetectionConstPtr msg, message_filters::Cache<Detection>* cache)
{
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<Detection>("detection_measured", 1);
  static int cnt = 0;
  if (++cnt < FREQ_FACTOR)
    return;
  else
    cnt = 0;
  
  DetectionConstPtr delayed = cache->getElemAfterTime(cache->getOldestTime());
  if (delayed != NULL)
  {
    Detection measured;
    measured.header = delayed->header;
    measured.valid = delayed->valid;
    if (delayed->valid)
    {
      measured.horz = quantizer(delayed->horz + noise_horz(gen), RESOLUTION);
      measured.vert = quantizer(delayed->vert + noise_vert(gen), RESOLUTION);
      measured.dist = delayed->dist + noise_dist(gen);  
    }
    pub.publish(measured);
  }
}

int main(int argc, char** argv)
{
  noise_horz = std::normal_distribution<double>(0, HORZ_VERT_STDDEV);
  noise_vert = std::normal_distribution<double>(0, HORZ_VERT_STDDEV);
  noise_dist = std::normal_distribution<double>(0, DIST_STDDEV);

  ros::init(argc, argv, "sensor_character");
  ros::NodeHandle nh;
  message_filters::Subscriber<Detection> sub(nh, "detection_groundtruth", 1);
  message_filters::Cache<Detection> cache(sub, CACHE_SIZE);
  cache.registerCallback(boost::bind(cacheCb, _1, &cache));
  ros::spin();
  return 0;
}