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
#include "MS561101BA.h"

// Conversion time in microseconds.
// Datasheet states that maximum conversion time is 9.04 ms at OSR = 4096.
// We choose 9500 us because it fits very well with a samplingrate of 100Hz.
// datasheet: http://www.hpinfotech.ro/MS5611-01BA03.pdf
#define CONVERSION_TIME		9500UL
#define SEA_PRESSURE 		1013.25f

bool MS561101BA::begin(uint8_t address, uint8_t OSR)
{
	addr = address;
	osr = OSR;
	lastPresConv = 0;
	lastTempConv = 0;
	D1 = 0;
	D2 = 0;
	STATE = AWAIT_D1;
	readPTratio = 1;

	// reset the device to populate its internal PROM registers
	reset(); 
	delay(50);
	
	// reads the PROM into object variables for later use
	if(!readPROM())
		return false;

	return true;
}

Vector3f MS561101BA::getTPA(bool compensate)
{
	// This state machine can read pressure more often than temperature.
	// 'MS561101BA_BAROM_PT_RATIO' defines the ratio pressure : temperature = MS561101BA_BAROM_PT_RATIO : 1 >= 1.
	// This is useful under the assumption that temperature does not change as fast as pressure.
	// The counter is 'readPTratio'.
	
	// final result
	Vector3f result;
	result.valid = false;
	
	// token to leave while loop, is set within the loop
	bool token = false;
	
	while(token == false)
	{
		switch(STATE)
		{
			case AWAIT_D1:
				if(getD1(osr))
				{
					readPTratio--;
					STATE = (readPTratio == 0) ? AWAIT_D2 : CALC_TPA;
				}
				else
				{					
					result.valid = false;
					token = true;
				}
				break;
				
			case AWAIT_D2:
				if(getD2(osr))
				{
					STATE = CALC_TPA;
				}
				else
				{
					result.valid = false;
					token = true;					
				}
				break;
				
			case CALC_TPA:
			{
				// converting D1 and D2 to temperature, pressure and altitude
				result = convertTPA(compensate);
				result.valid = true;
				
				// right after calculation start the next conversion for D1
				getD1(osr);
				readPTratio = (readPTratio == 0) ? MS561101BA_BAROM_PT_RATIO : readPTratio;
				STATE = AWAIT_D1;
				
				// values are now correct
				token = true;				
				break;
			}			
		}		
	}
	
	return result;
}

void MS561101BA::reset()
{
	Wire.beginTransmission(addr);
	Wire.write(MS561101BA_RESET);
	Wire.endTransmission();
}

bool MS561101BA::readPROM()
{
	for(int i=0; i<MS561101BA_PROM_REG_COUNT; i++)
	{
		Wire.beginTransmission(addr);
		Wire.write(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
		Wire.endTransmission();

		Wire.beginTransmission(addr);
		Wire.requestFrom(addr, (uint8_t) MS561101BA_PROM_REG_SIZE);
		if(Wire.available())
		{
			calib[i] = Wire.read() << 8 | Wire.read();
		}
		else
		{
			// error reading the PROM or communicating with the device
			return false;
		}
	}
	return true;
}

bool MS561101BA::getD1(uint8_t OSR)
{	
	bool ret_val = false;
	unsigned long now = micros();
	
	// check if conversion started and conversion time passed
	if(lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME)
	{
		lastPresConv = 0;
		D1 = getConversion(MS561101BA_D1 + OSR);
		ret_val = true;
	}
	else
	{
		// if no conversion running, start one
		if(lastPresConv == 0 && lastTempConv == 0)
		{
			startConversion(MS561101BA_D1 + OSR);
			lastPresConv = now;
		}
	}
	
	return ret_val;
}

bool MS561101BA::getD2(uint8_t OSR)
{
	bool ret_val = false;
	unsigned long now = micros();
	
	// check if conversion started and conversion time passed
	if(lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME)
	{
		lastTempConv = 0;
		D2 = getConversion(MS561101BA_D2 + OSR);
		ret_val = true;
	}
	else
	{
		// if no conversion running, start one
		if(lastTempConv == 0 && lastPresConv == 0)
		{
			startConversion(MS561101BA_D2 + OSR);
			lastTempConv = now;
		}
	}
	
	return ret_val;
}

void MS561101BA::startConversion(uint8_t command)
{
  // initialize pressure conversion
  Wire.beginTransmission(addr);
  Wire.write(command);
  Wire.endTransmission();
}

uint32_t MS561101BA::getConversion(uint8_t command)
{
  union {uint32_t val; uint8_t raw[4]; } conversion = {0};
  
	// start read sequence
	Wire.beginTransmission(addr);
	Wire.write(0);
	Wire.endTransmission();

	Wire.beginTransmission(addr);
	Wire.requestFrom(addr, (uint8_t) MS561101BA_D1D2_SIZE);
	if(Wire.available())
	{
		conversion.raw[2] = Wire.read();
		conversion.raw[1] = Wire.read();
		conversion.raw[0] = Wire.read();
	}
	else
	{
		conversion.val = -1;
	}

	return conversion.val;
}

// Why does the datasheet use 3 different integer types (uint32_t, int32_t and int64_t)?
// source: https://github.com/jarzebski/Arduino-MS5611
Vector3f MS561101BA::convertTPA(bool compensate)
{
	int32_t dT = D2 - (uint32_t)calib[4] * 256;
	int32_t TEMP = 2000 + ((int64_t) dT * calib[5]) / 8388608;
	int64_t OFF = (int64_t)calib[1] * 65536 + (int64_t)calib[3] * dT / 128;
	int64_t SENS = (int64_t)calib[0] * 32768 + (int64_t)calib[2] * dT / 256;

	if(compensate)
	{
		int32_t TEMP2 = (dT * dT) / (2 << 30);	
		
		int64_t OFF2 = 0;
		int64_t SENS2 = 0;

		if(TEMP < 2000)
		{
			OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
			SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
		}

		if(TEMP < -1500)
		{
			OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
			SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
		}
		
		TEMP = TEMP - TEMP2;

		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}

	uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;
	
	// convert to float, mbar and degrees Celcius
	float temperature = TEMP/100.0f;
	float pressure = P/100.0f;
	
	// calculate altitude in m relative to NN
	float altitute = ((pow((SEA_PRESSURE / pressure), 1/5.257) - 1.0) * (temperature + 273.15)) / 0.0065;

	return Vector3f(temperature, pressure, altitute);
}