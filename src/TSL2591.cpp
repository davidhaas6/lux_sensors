/*
 * Lux Sensors
 *
 * a program to read the inputs from the Adafruit TSL2561 and TSL2591 lux sensors
 *
 * David Haas        dhaas6@vt.edu          September 28th, 2018
 */

#include "lux_sensors/TSL2591.h"

TSL2591::TSL2591() {
  // Create I2C bus
	if((i2cHandler = open("/dev/i2c-1", O_RDWR)) < 0)
	{
		error = errno;
		printf("Failed to open the bus: %i \n", error);

		exit(1);
	}
	// Get I2C device, TSL2561 I2C address is 0x29
  if (ioctl(i2cHandler, I2C_SLAVE, TSL2591_ADDR) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    printf("Failed to open the bus. \n");
		exit(1);
  }

  setIntegration(TSL2591_INTEGRATIONTIME_100MS);
  setGain(TSL2591_GAIN_LOW);

  // Note: by default, the device is in power down mode on bootup
  disable();
}

bool TSL2591::write8(uint8_t writeRegister, uint8_t data) {
  char config[2];
  config[0] = writeRegister;
  config[1] = data;
  return write(i2cHandler, config, 2);
}

uint16_t TSL2591::read16(uint8_t readRegister) {
  uint8_t reg[1] = {readRegister};
	write(i2cHandler, reg, 1);

	uint8_t data[2] = {0};
	if(read(i2cHandler, data, 2) != 2) // reads in the data
	{
		printf("Erorr : Input/output Erorr \n");
	}

  // combine the bytes together
  uint16_t reading = (data[1]<<8) | data[0];
  return reading;
}


bool TSL2591::enable() {
  // Selects the enable register via the command register
  uint8_t cmd_addr = TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE;
  // enables the ALS and powers on the device
  uint8_t data = TSL2591_ENABLE_AEN | TSL2591_ENABLE_POWERON;

  enabled = write8(cmd_addr, data);
  if(!enabled)
    printf("Could not enable device!");
  return enabled;
}

bool TSL2591::disable() {
  if (!enabled)
    return true;
  // Selects the enable register via the command register
  uint8_t cmd_addr = TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE;
  // enables the ALS and powers off the device
  uint8_t data = TSL2591_ENABLE_POWEROFF;

  enabled = write8(cmd_addr, data);
  return enabled;
}

bool TSL2591::setIntegration(tsl2591IntegrationTime_t newTime) {
  int_time = newTime;

  uint8_t cmd_addr = TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL;
  uint8_t data = gain | int_time;

  enable();
  bool ret = write8(cmd_addr, data);
  disable();
  return ret;
}

bool TSL2591::setGain(tsl2591Gain_t newGain) {
  gain = newGain;

  uint8_t cmd_addr = TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL;
  uint8_t data = gain | int_time;

  enable();
  bool ret = write8(cmd_addr, data);
  disable();
  return ret;
}

sensorData_t TSL2591::getReadings() {
  uint16_t ch0;
  uint16_t ch1;

  enable();
  // Sleeps in microseconds to allow the ALS to integrate
  // 108 is the max integration time per step
  for (uint8_t d=0; d<=int_time; d++)
  {
    usleep((108 + int_time) * 1000);
  }

  ch0 = read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);
  ch1 = read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);

  disable();

  sensorData_t readings;
  readings.ch0 = ch0;
  readings.ch1 = ch1;

  return readings;
}

/************************************************************************/
/*!
    @brief  Calculates the visible Lux based on the two light sensors
    @param  ch0 Data from channel 0 (IR+Visible)
    @param  ch1 Data from channel 1 (IR)
    @returns Lux, based on AMS coefficients
*/
/**************************************************************************/
float TSL2591::getLux(sensorData_t readings)
{
  float    f_time, f_gain;
  float    cpl, lux;

  // Note: This algorithm is based on preliminary coefficients
  // provided by AMS and may need to be updated in the future

  switch (int_time)
  {
    case TSL2591_INTEGRATIONTIME_100MS :
      f_time = 100.0F;
      break;
    case TSL2591_INTEGRATIONTIME_200MS :
      f_time = 200.0F;
      break;
    case TSL2591_INTEGRATIONTIME_300MS :
      f_time = 300.0F;
      break;
    case TSL2591_INTEGRATIONTIME_400MS :
      f_time = 400.0F;
      break;
    case TSL2591_INTEGRATIONTIME_500MS :
      f_time = 500.0F;
      break;
    case TSL2591_INTEGRATIONTIME_600MS :
      f_time = 600.0F;
      break;
    default: // 100ms
      f_time = 100.0F;
      break;
  }

  switch (gain)
  {
    case TSL2591_GAIN_LOW :
      f_gain = 1.0F;
      break;
    case TSL2591_GAIN_MED :
      f_gain = 25.0F;
      break;
    case TSL2591_GAIN_HIGH :
      f_gain = 428.0F;
      break;
    case TSL2591_GAIN_MAX :
      f_gain = 9876.0F;
      break;
    default:
      f_gain = 1.0F;
      break;
  }

  // cpl = (ATIME * AGAIN) / DF
  cpl = (f_time * f_gain) / TSL2591_LUX_DF;

  // Alternate lux calculation 1
  // See: https://github.com/adafruit/Adafruit_TSL2591_Library/issues/14
  lux = ( ((float)readings.ch0 - (float)readings.ch1 )) * (1.0F - ((float)readings.ch1/(float)readings.ch0) ) / cpl;

  return lux;
}

float TSL2591::getLux() {
  return getLux(getReadings());
}