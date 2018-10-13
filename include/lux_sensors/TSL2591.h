/*
 * Name: TSL2591
 * Purpose: A linux driver for the Adafruit TSL2591 lux sensor
 * @author: David Haas
 * @since: 9/28/18
 */

#ifndef LUX_SENSOR_H
#define LUX_SENSOR_H

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

// I2C Slave Address
#define TSL2591_ADDR              (0x29)

///< 1010 0000: bits 7 and 5 for 'command normal'
#define TSL2591_COMMAND_BITS      (0xA0)

// Bit to enable the ALS register
#define TSL2591_ENABLE_AEN        (0x02)

//Bit to poweron the sensor
#define TSL2591_ENABLE_POWERON    (0x01)
#define TSL2591_ENABLE_POWEROFF   (0x00)

///< Lux cooefficient
#define TSL2591_LUX_DF            (408.0F)

// Stores the data from each channel
typedef struct {
	uint16_t ch0;
	uint16_t ch1;
}
sensorData_t;

/// TSL2591 Register map
enum
{
	TSL2591_REGISTER_ENABLE             = 0x00,// Enable register
	TSL2591_REGISTER_CONTROL            = 0x01,// Control register
	TSL2591_REGISTER_CHAN0_LOW          = 0x14,// Channel 0 data, low byte
	TSL2591_REGISTER_CHAN0_HIGH         = 0x15,// Channel 0 data, high byte
	TSL2591_REGISTER_CHAN1_LOW          = 0x16,// Channel 1 data, low byte
	TSL2591_REGISTER_CHAN1_HIGH         = 0x17,// Channel 1 data, high byte
};

typedef enum
{
	TSL2591_INTEGRATIONTIME_100MS     = 0x00,// 100 millis
	TSL2591_INTEGRATIONTIME_200MS     = 0x01,// 200 millis
	TSL2591_INTEGRATIONTIME_300MS     = 0x02,// 300 millis
	TSL2591_INTEGRATIONTIME_400MS     = 0x03,// 400 millis
	TSL2591_INTEGRATIONTIME_500MS     = 0x04,// 500 millis
	TSL2591_INTEGRATIONTIME_600MS     = 0x05,// 600 millis
}
tsl2591IntegrationTime_t;

typedef enum
{
	TSL2591_GAIN_LOW                  = 0x00,/// low gain (1x)
	TSL2591_GAIN_MED                  = 0x10,/// medium gain (25x)
	TSL2591_GAIN_HIGH                 = 0x20,/// medium gain (428x)
	TSL2591_GAIN_MAX                  = 0x30,/// max gain (9876x)
}
tsl2591Gain_t;


class TSL2591
{
private:
int i2cHandler;
bool enabled;
tsl2591IntegrationTime_t int_time;
tsl2591Gain_t gain;

bool write8(uint8_t writeRegister, uint8_t data);
uint16_t read16(uint8_t readRegister);

public:
int error;
TSL2591(tsl2591IntegrationTime_t nTime, tsl2591Gain_t nGain);
bool enable();
bool disable();
bool setIntegration(tsl2591IntegrationTime_t newTime);
bool setGain(tsl2591Gain_t newGain);
sensorData_t getReadings();
float getLux();
float getLux(sensorData_t readings);
};

#endif
