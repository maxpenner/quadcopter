/*

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// This library is a minimum library for MPU6050. 
// It's only task it to allow sensor calibration and retrieve the raw sensor data.
// Source (with modifications): https://github.com/jarzebski

#ifndef QC_MPU6050_H
#define QC_MPU6050_H

#include "vectorTypes.h"
#include "I2C.h"

// ============================= CONFIGURATION ==============================

// MPU6050 offset values (measured on 19.03.2016 with calibration sketch)
#define MPU6050_ACCEL_XOFFS		-2593
#define MPU6050_ACCEL_YOFFS		584
#define MPU6050_ACCEL_ZOFFS		2036
#define MPU6050_GYRO_XOFFS		0
#define MPU6050_GYRO_YOFFS		-50
#define MPU6050_GYRO_ZOFFS		22

// ============================ CONFIGURATION END ===========================

#define MPU6050_ADDRESS				(0x68) 	// default address, can be set to 0x69 by connecting pin AD0 to supply voltage
#define MPU6050_REG_ACCEL_XOFFS_H	(0x06) 	// undocumented, but can and must be set with calibration values
#define MPU6050_REG_ACCEL_YOFFS_H	(0x08) 	// "
#define MPU6050_REG_ACCEL_ZOFFS_H	(0x0A) 	// "
#define MPU6050_REG_GYRO_XOFFS_H	(0x13) 	// "
#define MPU6050_REG_GYRO_YOFFS_H	(0x15) 	// "
#define MPU6050_REG_GYRO_ZOFFS_H	(0x17) 	// "
#define MPU6050_REG_CONFIG			(0x1A)	// digital low pass filter (DLPF) setting
#define MPU6050_REG_GYRO_CONFIG		(0x1B) 	// gyroscope configuration
#define MPU6050_REG_ACCEL_CONFIG	(0x1C) 	// accelerometer configuration
#define MPU6050_REG_INT_PIN_CFG		(0x37) 	// interrupt pin and bypass enable configuration
#define MPU6050_REG_ACCEL_XOUT_H	(0x3B)	// accelerometer output register
#define MPU6050_REG_TEMP_OUT_H		(0x41)	// temperature output register
#define MPU6050_REG_GYRO_XOUT_H		(0x43)	// gyroscope output register
#define MPU6050_REG_USER_CTRL		(0x6A) 	// disable I2C master mode
#define MPU6050_REG_PWR_MGMT_1		(0x6B) 	// disable sleep mode and set internal clock source
#define MPU6050_REG_WHO_AM_I		(0x75) 	// check if sensor is valid

typedef enum
{
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
}mpu6050_clockSource_t;

typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
}mpu6050_dps_t;

typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00
}mpu6050_range_t;

typedef enum
{
    MPU6050_DELAY_3MS             = 0b11,
    MPU6050_DELAY_2MS             = 0b10,
    MPU6050_DELAY_1MS             = 0b01,
    MPU6050_NO_DELAY              = 0b00
}mpu6050_onDelay_t;

typedef enum
{
    MPU6050_DHPF_HOLD             = 0b111,
    MPU6050_DHPF_0_63HZ           = 0b100,
    MPU6050_DHPF_1_25HZ           = 0b011,
    MPU6050_DHPF_2_5HZ            = 0b010,
    MPU6050_DHPF_5HZ              = 0b001,
    MPU6050_DHPF_RESET            = 0b000
}mpu6050_dhpf_t;

typedef enum
{
    MPU6050_DLPF_6                = 0b110,
    MPU6050_DLPF_5                = 0b101,
    MPU6050_DLPF_4                = 0b100,
    MPU6050_DLPF_3                = 0b011,
    MPU6050_DLPF_2                = 0b010,
    MPU6050_DLPF_1                = 0b001,
    MPU6050_DLPF_0                = 0b000
}mpu6050_dlpf_t;

class MPU6050
{
    public:
	
		// initialize sensor
		bool begin(mpu6050_dps_t scale = MPU6050_SCALE_500DPS, mpu6050_range_t range = MPU6050_RANGE_4G, int mpua = MPU6050_ADDRESS);

		// set internal or external clock source
		mpu6050_clockSource_t getClockSource(void);
		void setClockSource(mpu6050_clockSource_t source);
		
		// set and get scale and range settings
		mpu6050_dps_t getScale(void);
		void setScale(mpu6050_dps_t scale);
		mpu6050_range_t getRange(void);
		void setRange(mpu6050_range_t range);
		
		// digital low/high pass filter setting
		void setDLPFMode(mpu6050_dlpf_t dlpf);
		void setDHPFMode(mpu6050_dhpf_t dhpf);

		// sleep mode, I2C mode and bypass enabling
		bool getSleepEnabled(void);
		void setSleepEnabled(bool state);
		bool getI2CMasterModeEnabled(void);
		void setI2CMasterModeEnabled(bool state);
		bool getI2CBypassEnabled(void);
		void setI2CBypassEnabled(bool state);

		// gyroscope offset (undocumented): can be measured with calibration sketch
		int16_t getGyroOffsetX(void);
		int16_t getGyroOffsetY(void);
		int16_t getGyroOffsetZ(void);
		void setGyroOffsetX(int16_t offset);
		void setGyroOffsetY(int16_t offset);
		void setGyroOffsetZ(int16_t offset);

		// accelerometer offset (undocumented): can be measured with calibration sketch
		int16_t getAccelOffsetX(void);
		int16_t getAccelOffsetY(void);
		int16_t getAccelOffsetZ(void);
		void setAccelOffsetX(int16_t offset);
		void setAccelOffsetY(int16_t offset);
		void setAccelOffsetZ(int16_t offset);
		
		// sensor temperature
		float readTemperature(void);

		// read raw gyroscope value
		Vector3f readRawGyro(void);		// register integer values converted to float
		Vector3f readDpsGyro(void);		// degrees per second
		Vector3f readRpsGyro(void);		// radians per second

		// read raw accelerometer value
		Vector3f readRawAccel(void);	// register integer values converted to float
		Vector3f readNpkgAccel(void);	// Newton per kilogramm, e.g. x=0, y=0 and z=9.81
		Vector3f readGAccel(void);		// multiples on 1G, e.g. x=0, y=0 and z=1.0

    private:

		float dpsPerDigit;
		float rangePerDigit;
	
		I2C i2c;
};

#endif