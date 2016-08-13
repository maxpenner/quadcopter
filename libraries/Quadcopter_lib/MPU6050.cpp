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

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

#define DEG2RAD 0.017453292f
#define RAD2DEG 57.29577951f
#define GRAVITY 9.80665f		// average value in Europe

bool MPU6050::begin(mpu6050_dps_t scale, mpu6050_range_t range, int mpua)
{
	i2c.setAddr(mpua);
	
    if(i2c.readRegister8(MPU6050_REG_WHO_AM_I) != 0x68)
		return false;
	
	// set minimum settings for sensor
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setRange(range);
	setDLPFMode(MPU6050_DLPF_1);
	
	// be an I2C slave and bypass auxiliary I2C bus
	setI2CMasterModeEnabled(false);
	setI2CBypassEnabled(true);
	
    // set offsets
	setGyroOffsetX(MPU6050_GYRO_XOFFS);
	setGyroOffsetY(MPU6050_GYRO_YOFFS);
	setGyroOffsetZ(MPU6050_GYRO_ZOFFS);
	setAccelOffsetX(MPU6050_ACCEL_XOFFS);
	setAccelOffsetY(MPU6050_ACCEL_YOFFS);
	setAccelOffsetZ(MPU6050_ACCEL_ZOFFS);	

    // disable sleep mode
    setSleepEnabled(false);

	// ##########################################################
	// TODO: EXPLAIN BUG
	// HAS TO BE SET AT LAST
    setScale(scale);
	// ##########################################################

    return true;
}

mpu6050_clockSource_t MPU6050::getClockSource(void)
{
    uint8_t value;
    value = i2c.readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b00000111;
    return (mpu6050_clockSource_t)value;
}

void MPU6050::setClockSource(mpu6050_clockSource_t source)
{
    uint8_t value;
    value = i2c.readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    i2c.writeRegister8(MPU6050_REG_PWR_MGMT_1, value);
}

mpu6050_dps_t MPU6050::getScale(void)
{
    uint8_t value;
    value = i2c.readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t)value;
}

void MPU6050::setScale(mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
		case MPU6050_SCALE_250DPS:
			dpsPerDigit = .007633f;
			break;
		case MPU6050_SCALE_500DPS:
			dpsPerDigit = .015267f;
			break;
		case MPU6050_SCALE_1000DPS:
			dpsPerDigit = .030487f;
			break;
		case MPU6050_SCALE_2000DPS:
			dpsPerDigit = .060975f;
			break;
		default:
			break;
    }

    value = i2c.readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    i2c.writeRegister8(MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_range_t MPU6050::getRange(void)
{
    uint8_t value;
    value = i2c.readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t)value;
}

void MPU6050::setRange(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
		case MPU6050_RANGE_2G:
			rangePerDigit = .000061f;
			break;
		case MPU6050_RANGE_4G:
			rangePerDigit = .000122f;
			break;
		case MPU6050_RANGE_8G:
			rangePerDigit = .000244f;
			break;
		case MPU6050_RANGE_16G:
			rangePerDigit = .0004882f;
			break;
		default:
			break;
    }

    value = i2c.readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    i2c.writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

void MPU6050::setDLPFMode(mpu6050_dlpf_t dlpf)
{
    uint8_t value;
    value = i2c.readRegister8(MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    i2c.writeRegister8(MPU6050_REG_CONFIG, value);
}

void MPU6050::setDHPFMode(mpu6050_dhpf_t dhpf)
{
    uint8_t value;
    value = i2c.readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11111000;
    value |= dhpf;
    i2c.writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

bool MPU6050::getSleepEnabled(void)
{
    return i2c.readRegisterBit(MPU6050_REG_PWR_MGMT_1, 6);
}

void MPU6050::setSleepEnabled(bool state)
{
    i2c.writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

bool MPU6050::getI2CMasterModeEnabled(void)
{
    return i2c.readRegisterBit(MPU6050_REG_USER_CTRL, 5);
}

void MPU6050::setI2CMasterModeEnabled(bool state)
{
    i2c.writeRegisterBit(MPU6050_REG_USER_CTRL, 5, state);
}

bool MPU6050::getI2CBypassEnabled(void)
{
    return i2c.readRegisterBit(MPU6050_REG_INT_PIN_CFG, 1);
}

void MPU6050::setI2CBypassEnabled(bool state)
{
    return i2c.writeRegisterBit(MPU6050_REG_INT_PIN_CFG, 1, state);
}

int16_t MPU6050::getGyroOffsetX(void)
{
    return i2c.readRegister16(MPU6050_REG_GYRO_XOFFS_H);
}

int16_t MPU6050::getGyroOffsetY(void)
{
    return i2c.readRegister16(MPU6050_REG_GYRO_YOFFS_H);
}

int16_t MPU6050::getGyroOffsetZ(void)
{
    return i2c.readRegister16(MPU6050_REG_GYRO_ZOFFS_H);
}

void MPU6050::setGyroOffsetX(int16_t offset)
{
    i2c.writeRegister16(MPU6050_REG_GYRO_XOFFS_H, offset);
}

void MPU6050::setGyroOffsetY(int16_t offset)
{
    i2c.writeRegister16(MPU6050_REG_GYRO_YOFFS_H, offset);
}

void MPU6050::setGyroOffsetZ(int16_t offset)
{
    i2c.writeRegister16(MPU6050_REG_GYRO_ZOFFS_H, offset);
}

int16_t MPU6050::getAccelOffsetX(void)
{
    return i2c.readRegister16(MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t MPU6050::getAccelOffsetY(void)
{
    return i2c.readRegister16(MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t MPU6050::getAccelOffsetZ(void)
{
    return i2c.readRegister16(MPU6050_REG_ACCEL_ZOFFS_H);
}

void MPU6050::setAccelOffsetX(int16_t offset)
{
    i2c.writeRegister16(MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void MPU6050::setAccelOffsetY(int16_t offset)
{
    i2c.writeRegister16(MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void MPU6050::setAccelOffsetZ(int16_t offset)
{
    i2c.writeRegister16(MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

float MPU6050::readTemperature(void)
{
    int16_t T;
    T = i2c.readRegister16(MPU6050_REG_TEMP_OUT_H);
	
    return (float)T/340 + 36.53;
}

Vector3f MPU6050::readRawGyro(void)
{
    Wire.beginTransmission(i2c.getAddr());
	Wire.write(MPU6050_REG_GYRO_XOUT_H);
    Wire.endTransmission();

    Wire.beginTransmission(i2c.getAddr());
    Wire.requestFrom(i2c.getAddr(), 6);

    while (Wire.available() < 6)
	{
	};

	uint8_t xha = Wire.read();
	uint8_t xla = Wire.read();
    uint8_t yha = Wire.read();
	uint8_t yla = Wire.read();
	uint8_t zha = Wire.read();
	uint8_t zla = Wire.read();

	// pack 2 bytes in one signed 16 bit number
	int16_t rg_x = xha << 8 | xla;
	int16_t rg_y = yha << 8 | yla;
	int16_t rg_z = zha << 8 | zla;

	// convert signed 16 bit integer to float
    return Vector3f((float) rg_x, (float) rg_y, (float) rg_z);
}

Vector3f MPU6050::readDpsGyro(void)
{
	Vector3f rawG = readRawGyro();

    return Vector3f(rawG.x*dpsPerDigit, rawG.y*dpsPerDigit, rawG.z*dpsPerDigit);
}

Vector3f MPU6050::readRpsGyro(void)
{
	Vector3f convG = readDpsGyro();

    return Vector3f(convG.x*DEG2RAD, convG.y*DEG2RAD, convG.z*DEG2RAD);
}

Vector3f MPU6050::readRawAccel(void)
{
    Wire.beginTransmission(i2c.getAddr());
	Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    Wire.endTransmission();

    Wire.beginTransmission(i2c.getAddr());
    Wire.requestFrom(i2c.getAddr(), 6);

    while (Wire.available() < 6)
	{		
	};

	uint8_t xha = Wire.read();
	uint8_t xla = Wire.read();
	uint8_t yha = Wire.read();
	uint8_t yla = Wire.read();
	uint8_t zha = Wire.read();
	uint8_t zla = Wire.read();

	// pack 2 bytes in one signed 16 bit number
	int16_t ra_x = xha << 8 | xla;
	int16_t ra_y = yha << 8 | yla;
	int16_t ra_z = zha << 8 | zla;

	// convert signed 16 bit integer to float
    return Vector3f((float) ra_x, (float) ra_y, (float) ra_z);
}

Vector3f MPU6050::readNpkgAccel(void)
{
	Vector3f rawA = readRawAccel();

	float tmp = rangePerDigit * GRAVITY;

    return Vector3f(rawA.x*tmp, rawA.y*tmp, rawA.z*tmp);
}

Vector3f MPU6050::readGAccel(void)
{
	Vector3f rawA = readRawAccel();

    return Vector3f(rawA.x*rangePerDigit, rawA.y*rangePerDigit, rawA.z*rangePerDigit);
}