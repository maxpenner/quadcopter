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

// This library is a minimum library for I2C bus access with Arduino.
// source (with modifications): https://github.com/jarzebski

#ifndef QC_I2C_H
#define QC_I2C_H

class I2C
{
	public:

		I2C();
		I2C(int address);
		~I2C();
		
		// set and get device address
		int getAddr();
		void setAddr(int address);
		
		// read and write single byte
		uint8_t readRegister8(uint8_t reg);
		void writeRegister8(uint8_t reg, uint8_t value);

		// read and write two consecutive bytes
		int16_t readRegister16(uint8_t reg);
		void writeRegister16(uint8_t reg, int16_t value);

		// read and write single bit of a byte
		bool readRegisterBit(uint8_t reg, uint8_t pos);
		void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);
		
	private:
	
		int deviceAddress;
};

#endif