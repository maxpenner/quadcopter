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

#include <Wire.h>
#include "I2C.h"

I2C::I2C()
{
}

I2C::I2C(int address)
{
	deviceAddress = address;
}

I2C::~I2C()
{
}

int I2C::getAddr()
{
	return deviceAddress;
}

void I2C::setAddr(int address)
{
	deviceAddress = address;
}

uint8_t I2C::readRegister8(uint8_t reg)
{
    uint8_t value;

    Wire.beginTransmission(deviceAddress);
	Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(deviceAddress);
    Wire.requestFrom(deviceAddress, 1);
	
    while(!Wire.available())
	{
	};
	
	value = Wire.read();
    Wire.endTransmission();

    return value;
}

void I2C::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(deviceAddress);
	Wire.write(reg);
	Wire.write(value);
    Wire.endTransmission();
}

int16_t I2C::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire.beginTransmission(deviceAddress);
	Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(deviceAddress);
    Wire.requestFrom(deviceAddress, 2);
	
    while(!Wire.available())
	{
	};
	
	uint8_t vha = Wire.read();
	uint8_t vla = Wire.read();
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

void I2C::writeRegister16(uint8_t reg, int16_t value)
{
    Wire.beginTransmission(deviceAddress);
	Wire.write(reg);
	Wire.write((uint8_t)(value >> 8));
	Wire.write((uint8_t)value);
    Wire.endTransmission();
}

bool I2C::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
	
    return ((value >> pos) & 1);
}

void I2C::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if(state)
        value |= (1 << pos);
	else 
        value &= ~(1 << pos);

    writeRegister8(reg, value);
}