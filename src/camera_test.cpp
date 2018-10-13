/*
 * Name: camera_testing_node
 * Purpose: A node for testing the effects of lux and exposure time on the camera
 * @author: David Haas
 * @since: 10/12/18
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include "ros/ros.h"
#include "sensor_msgs/Illuminance.h"
#pragma GCC diagnostic pop

float lux = 0;
float exp_times[] = {1.08,2.16,5.41,10,20,39,78,156,312,625};

void luxCallback(const sensor_msgs::Illuminance::ConstPtr & msg) {
  ROS_INFO("Lux: %f", msg->illuminance);
  lux = msg -> illuminance;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_exposure_testing");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("TSL2591_lux", 100, luxCallback);

  if (argc == 1) {
  ros::spinOnce();
  for(float exp_time : exp_times) {
    //camera.setExposure(false, (int) exp_time, DEFAULT_BRIGHTNESS);
    //camera.capture(false, ros::Time::now());
    usleep(exp_time * 100); // sleeps in microseconds
  }
}
  else if (argc == 2) {
    //camera.setExposure(false, (int) std::stoi(argv[1]), DEFAULT_BRIGHTNESS);
    //camera.capture(false, ros::Time::now());
  }
}
