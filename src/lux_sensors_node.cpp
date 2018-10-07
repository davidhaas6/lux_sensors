/*
 * Name: lux_sensors_node
 * Purpose: A node for the TSL2591 linux driver
 * @author: David Haas
 * @since: 9/28/18
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include "ros/ros.h"
#include "sensor_msgs/Illuminance.h"
#pragma GCC diagnostic pop

#include "lux_sensors/TSL2591.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lux_sensors");
  ros::NodeHandle n;

  // Node publishes raw data from each channel as well as the lux calculated
  ros::Publisher tsl2591_lux_pub = n.advertise<sensor_msgs::Illuminance>("TSL2591_lux", 1000);
  ros::Publisher tsl2591_vis_pub = n.advertise<sensor_msgs::Illuminance>("TSL2591_visible", 1000);
  ros::Publisher tsl2591_ir_pub = n.advertise<sensor_msgs::Illuminance>("TSL2591_ir", 1000);

  ros::Rate loop_rate(8);

  // Integration time is 100ms to get frequent updates from sensor
  // Gain is low (1x) because it will be under full-daylight conditions
  TSL2591 tsl2591(TSL2591_INTEGRATIONTIME_100MS, TSL2591_GAIN_LOW);

  // Read, publish, log data
  while (ros::ok())
  {
    sensorData_t readings = tsl2591.getReadings();

    sensor_msgs::Illuminance tsl2591_lux_msg;
    sensor_msgs::Illuminance tsl2591_vis_msg;
    sensor_msgs::Illuminance tsl2591_ir_msg ;

    tsl2591_lux_msg.illuminance = tsl2591.getLux(readings);
    tsl2591_lux_msg.header.stamp = ros::Time::now();

    tsl2591_vis_msg.illuminance = readings.ch0;
    tsl2591_vis_msg.header.stamp = ros::Time::now();

    tsl2591_ir_msg.illuminance = readings.ch1;
    tsl2591_ir_msg.header.stamp = ros::Time::now();

    tsl2591_lux_pub.publish(tsl2591_lux_msg);
    tsl2591_vis_pub.publish(tsl2591_vis_msg);
    tsl2591_ir_pub.publish(tsl2591_ir_msg);

    ROS_INFO("Lux: %f", tsl2591_lux_msg.illuminance);
    ROS_INFO("Visible: %f", tsl2591_vis_msg.illuminance);
    ROS_INFO("Infared: %f", tsl2591_ir_msg.illuminance);

    loop_rate.sleep();
  }

}
