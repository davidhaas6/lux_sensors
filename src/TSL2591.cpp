/*
 * Name: TSL2591
 * Purpose: A linux driver for the Adafruit TSL2591 lux sensor
 * @author: David Haas
 * @since: 9/28/18
 */

#include "lux_sensors/TSL2591.h"

/**
 * Initializes i2c communication as well as gain and integration time
 */
TSL2591::TSL2591(tsl2591IntegrationTime_t nTime, tsl2591Gain_t nGain) {
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


	int_time = nTime;
	gain = nGain;
	// Only sets integration time b/c both are written in that method
	setIntegration(int_time);

	// Note: by default, the device is in power down mode on bootup
	disable();
}

/**
 * Writes 8 bits to an i2c device register
 * @param writeRegister The register on the device to write to
 * @param data The data to write to the register
 * @return A bool denoting whether the device was successfully written to
 */
bool TSL2591::write8(uint8_t writeRegister, uint8_t data) {
	char message[2];
	message[0] = writeRegister;
	message[1] = data;
	return write(i2cHandler, message, 2);
}

/**
 * Reads 16 bits from an i2c device register
 * @param readRegister The register on the device to read from
 * @return The data read from the register
 */
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

/**
 * Powers on the device and enables the ALS (ambient light sensor)
 * @return A bool denoting whether the device was successfully enabled
 */
bool TSL2591::enable() {
	// Selects the enable register via the command register
	uint8_t cmd_addr = TSL2591_COMMAND_BITS | TSL2591_REGISTER_ENABLE;
	// enables the ALS and powers on the device
	uint8_t data = TSL2591_ENABLE_AEN | TSL2591_ENABLE_POWERON;

	enabled = write8(cmd_addr, data);
	if(!enabled)
		printf("Could not enable device!");
	return enabled;
}

/**
 * Powers off the device
 * @return A bool denoting whether the device was successfully disabled
 */
bool TSL2591::disable() {
	if (!enabled)
		return true;

	// Selects the enable register via the command register
	uint8_t cmd_addr = TSL2591_COMMAND_BITS | TSL2591_REGISTER_ENABLE;
	// enables the ALS and powers off the device
	uint8_t data = TSL2591_ENABLE_POWEROFF;

	enabled = write8(cmd_addr, data);
	return enabled;
}

/**
 * Sets the integration time (ATIME) of the ALS on the device
 * @param newTime The hex code of the desired integration time
 * @return A bool denoting whether the integration time was successfully changed
 */
bool TSL2591::setIntegration(tsl2591IntegrationTime_t newTime) {
	int_time = newTime;

	// Selects the control register
	uint8_t cmd_addr = TSL2591_COMMAND_BITS | TSL2591_REGISTER_CONTROL;

	// Sets the current gain and the new time
	uint8_t data = gain | int_time;

	// Write data
	enable();
	bool ret = write8(cmd_addr, data);
	disable();
	return ret;
}

/**
 * Sets the gain (AGAIN) of the ALS on the device
 * @param newGain The hex code of the desired gain
 * @return A bool denoting whether the gain was successfully changed
 */
bool TSL2591::setGain(tsl2591Gain_t newGain) {
	gain = newGain;

	// Selects the control register
	uint8_t cmd_addr = TSL2591_COMMAND_BITS | TSL2591_REGISTER_CONTROL;

	// Sets the current gain and the new time
	uint8_t data = gain | int_time;

	// Write data
	enable();
	bool ret = write8(cmd_addr, data);
	disable();
	return ret;
}

/**
 * Reads the IR and visible ALS data from the device's registers
 * @return A struct containing each of the channel's data
 */
sensorData_t TSL2591::getReadings() {
	uint16_t ch0; // visible channel
	uint16_t ch1; // IR channel

	enable();
	// Sleeps (in microseconds) to allow the ALS to integrate
	// 108 is the max integration time per step as per the datasheet
	for (uint8_t d=0; d<=int_time; d++)
	{
		usleep((108 + int_time) * 1000);
	}

	// Reads in data
	ch0 = read16(TSL2591_COMMAND_BITS | TSL2591_REGISTER_CHAN0_LOW);
	ch1 = read16(TSL2591_COMMAND_BITS | TSL2591_REGISTER_CHAN1_LOW);

	disable();

	sensorData_t readings;
	readings.ch0 = ch0;
	readings.ch1 = ch1;

	return readings;
}

/**
 * Calculates the visible lux based on the IR and visible sensors
 * @author Adafruit
 * @param readings The visible and IR data from the device's registers
 * @return Lux, based on AMS coefficients
 */
float TSL2591::getLux(sensorData_t readings)
{
	float f_time, f_gain;
	float cpl, lux;

	// Sets the gain and integration time to their decimal correspondents
	switch (int_time)
	{
	case TSL2591_INTEGRATIONTIME_100MS:
		f_time = 100.0F;
		break;
	case TSL2591_INTEGRATIONTIME_200MS:
		f_time = 200.0F;
		break;
	case TSL2591_INTEGRATIONTIME_300MS:
		f_time = 300.0F;
		break;
	case TSL2591_INTEGRATIONTIME_400MS:
		f_time = 400.0F;
		break;
	case TSL2591_INTEGRATIONTIME_500MS:
		f_time = 500.0F;
		break;
	case TSL2591_INTEGRATIONTIME_600MS:
		f_time = 600.0F;
		break;
	default: // 100ms
		f_time = 100.0F;
		break;
	}

	switch (gain)
	{
	case TSL2591_GAIN_LOW:
		f_gain = 1.0F;
		break;
	case TSL2591_GAIN_MED:
		f_gain = 25.0F;
		break;
	case TSL2591_GAIN_HIGH:
		f_gain = 428.0F;
		break;
	case TSL2591_GAIN_MAX:
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

/**
 * Calculates the visible lux based on the IR and visible sensors
 * @return Lux, based on AMS coefficients
 */
float TSL2591::getLux() {
	return getLux(getReadings());
}
