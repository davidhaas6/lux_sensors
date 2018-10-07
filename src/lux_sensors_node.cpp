/*
 * Lux Data Node
 *
 * Publishes the lux data to ROS
 *
 * David Haas        dhaas6@vt.edu          September 28th, 2018
 */

#include "ros/ros.h"
#include "sensor_msgs/Illuminance.h"

#include "lux_sensors/TSL2591.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lux_sensors");
  ros::NodeHandle n;
  
  ros::Publisher tsl2591_lux_pub = n.advertise<sensor_msgs::Illuminance>("TSL2591_lux", 1000);
  ros::Publisher tsl2591_vis_pub = n.advertise<sensor_msgs::Illuminance>("TSL2591_visible", 1000);
  ros::Publisher tsl2591_ir_pub = n.advertise<sensor_msgs::Illuminance>("TSL2591_ir", 1000);

  ros::Rate loop_rate(8);

  TSL2591 tsl2591;
  tsl2591.setIntegration(TSL2591_INTEGRATIONTIME_100MS);
  tsl2591.setGain(TSL2591_GAIN_LOW);

  // Read, publish, log data
  while (ros::ok())
  {
    sensorData_t readings = tsl2591.getReadings();

    sensor_msgs::Illuminance tsl2591_lux_msg = tsl2591.getLux(readings);
    sensor_msgs::Illuminance tsl2591_vis_msg = readings.ch0;
    sensor_msgs::Illuminance tsl2591_ir_msg = readings.ch1;

    tsl2591_lux_pub.publish(tsl2591_lux_msg);
    tsl2591_vis_pub.publish(tsl2591_vis_msg);
    tsl2591_ir_pub.publish(tsl2591_ir_msg);

    ROS_INFO("Lux: %f", tsl2591_lux_msg.illuminance);
    ROS_INFO("Visible: %f", tsl2591_vis_msg.illuminance);
    ROS_INFO("Infared: %f", tsl2591_ir_msg.illuminance);

    loop_rate.sleep();
  }

}
