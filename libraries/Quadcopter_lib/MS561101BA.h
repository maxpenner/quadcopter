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

// This library is a minimum library for MS5611. 
// It's only task it to retrieve the raw sensor data.
// source (with modifications): https://github.com/jarzebski

#ifndef QC_MS561101BA_h
#define QC_MS561101BA_h

#include "vectorTypes.h"

// ============================= CONFIGURATION ==============================

// Barometer ratio between reading the pressure and the temperature (increases the number of valid values).
// The temperature does not change as fast as the pressure.
#define MS561101BA_BAROM_PT_RATIO	20

// ============================ CONFIGURATION END ===========================

#define MS561101BA_ADDR_CSB_HIGH  	0x76   	// CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR_CSB_LOW		0x77   	// CBR=0 0x77 I2C address when CSB is connected to LOW (GND)
#define MS561101BA_D1				0x40	// registers of the device
#define MS561101BA_D2 				0x50
#define MS561101BA_RESET 			0x1E
#define MS561101BA_D1D2_SIZE		3		// D1 and D2 result size (bytes)
#define MS561101BA_OSR_256 			0x00	// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_512 			0x02
#define MS561101BA_OSR_1024			0x04
#define MS561101BA_OSR_2048			0x06
#define MS561101BA_OSR_4096			0x08
#define MS561101BA_PROM_BASE_ADDR	0xA2 	// by adding ints from 0 to 6 we can read all the prom configuration values. 
											// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 	6 		// number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 	2 		// size in bytes of a prom registry.

class MS561101BA
{
	public:
	
		// initialization function
		bool begin(uint8_t address, uint8_t OSR);
		
		// data extraction function (x=temperature, y=pressure, z=altitude, valid=data validity)
		Vector3f getTPA(bool compensate);
		
	private:
	
		// internal initialization functions
		void reset();
		bool readPROM();
		
		// chip access and data extraction functions
		bool getD1(uint8_t OSR);
		bool getD2(uint8_t OSR);
		void startConversion(uint8_t command);
		uint32_t getConversion(uint8_t command);
		
		// converting D1 and D2 to temperature, pressure and altitude
		Vector3f convertTPA(bool compensate);
		
		// variables for calculation of temperature, pressure and altitude
		uint8_t addr, osr;
		uint16_t calib[MS561101BA_PROM_REG_COUNT];
		uint32_t lastPresConv, lastTempConv;
		uint32_t D1, D2;
		
		// state machine variables for function 'getTPA()'
		enum states{AWAIT_D1, AWAIT_D2, CALC_TPA};
		states STATE;
		int readPTratio;
};

#endif