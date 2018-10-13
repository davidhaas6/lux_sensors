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

void luxCallback(const sensor_msgs::Illuminance::ConstPtr & msg) {
  ROS_INFO("Lux: %f", msg->illuminance);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_exposure_testing");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("TSL2591_lux", 100, luxCallback);
}
