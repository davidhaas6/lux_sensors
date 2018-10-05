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
  ros::Publisher hdr_2591_pub = n.advertise<sensor_msgs::Illuminance>("lux_hdr", 1000);
  // ros::Publisher ldr_2561_pub = n.advertise<sensor_msgs::Illuminance>("lux_ldr", 1000);
  ros::Rate loop_rate(8);


  // // The high dynamic range (HDR) sensor
  // Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591);
  // tsl.setGain(TSL2591_GAIN_LOW); // 1x gain for bright light
  // // How long the sensor collects light data for - ranges from 100-500ms
  // // higher integration time -> higher resolution data
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  //
  // // The low dynamic range (LDR) sensor
  // Adafruit_TSL2561 tsl2561 = Adafruit_TSL2561(TSL2561_ADDR_FLOAT, 12345);
  // tsl.setGain(TSL2561_GAIN_1X); // 1x gain for bright light
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS); // 13, 101 or 402 ms
  TSL2591 tsl2591;
  tsl2591.setIntegration(TSL2591_INTEGRATIONTIME_100MS);
  tsl2591.setGain(TSL2591_GAIN_LOW);



  while (ros::ok())
  {
    sensor_msgs::Illuminance hdr_msg; // Message for the TSL2591
    //sensor_msgs::Illuminance ldr_msg; // Message for the TSL2561

    hdr_msg.illuminance = tsl2591.getLux();
    ROS_INFO("%f", hdr_msg.illuminance);
    hdr_2591_pub.publish(hdr_msg);
    // // Reads and publishes the HDR lux
    // tsl2591.getEvent(&event);
    // if(event.light) { // If
    //   hdr_msg.illuminance = event.light;
    //   hdr_2591_pub.publish(hdr_msg);
    //   ROS_INFO("%f", hdr_msg.illuminance);
    // } else {
    //   ROS_WARN("TSL2591 Reading == 0, Sensor is likely over saturated");
    // }
    //
    // // Reads and publishes the HDR lux
    // tsl2561.getEvent(&event);
    // if(event.light) { // If
    //   ldr_msg.illuminance = event.light;
    //   ldr_2561_pub.publish(ldr_msg);
    //   ROS_INFO("%f", ldr_msg.illuminance);
    // } else {
    //   ROS_WARN("TSL2561 Reading == 0, Sensor is likely over saturated");
    // }

    loop_rate.sleep();
  }

}
