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
#include "HMC5883L.h"

bool HMC5883L::begin()
{
	i2c.setAddr(HMC5883L_ADDRESS);
	
    if((i2c.readRegister8(HMC5883L_REG_IDENT_A) != 0x48) || (i2c.readRegister8(HMC5883L_REG_IDENT_B) != 0x34) || (i2c.readRegister8(HMC5883L_REG_IDENT_C) != 0x33))
		return false;

	// set minimum settings for sensor
    setRange(HMC5883L_RANGE_1_3GA);
	setMeasurementMode(HMC5883L_CONTINUOUS);
	setDataRate(HMC5883L_DATARATE_15HZ);
	setSamples(HMC5883L_SAMPLES_8);

    return true;
}

Vector3f HMC5883L::readRaw(void)
{
	Vector3f tmp;
	
    tmp.x = (float)i2c.readRegister16(HMC5883L_REG_OUT_X_M);
    tmp.y = (float)i2c.readRegister16(HMC5883L_REG_OUT_Y_M);
    tmp.z = (float)i2c.readRegister16(HMC5883L_REG_OUT_Z_M);

    return tmp;
}

Vector3f HMC5883L::readNormalize(void)
{
	Vector3f tmp;
	
    tmp.x = ((float)i2c.readRegister16(HMC5883L_REG_OUT_X_M)) * mgPerDigit;
    tmp.y = ((float)i2c.readRegister16(HMC5883L_REG_OUT_Y_M)) * mgPerDigit;
    tmp.z = ((float)i2c.readRegister16(HMC5883L_REG_OUT_Z_M)) * mgPerDigit;

    return tmp;
}

Vector3f HMC5883L::readCalibrated(void)
{
	Vector3f tmp = readNormalize();
	
	// remove bias
	tmp.x -= HMC5883L_BIAS_X;
	tmp.y -= HMC5883L_BIAS_Y;
	tmp.z -= HMC5883L_BIAS_Z;
	
	// form sphere from ellipsoid
	Vector3f tmp2;
	tmp2.x = HMC5883L_A11*tmp.x + HMC5883L_A12*tmp.y + HMC5883L_A13*tmp.z;
	tmp2.y = HMC5883L_A21*tmp.x + HMC5883L_A22*tmp.y + HMC5883L_A23*tmp.z;
	tmp2.z = HMC5883L_A31*tmp.x + HMC5883L_A32*tmp.y + HMC5883L_A33*tmp.z;
	
	return tmp2;
}

hmc5883l_range_t HMC5883L::getRange(void)
{
    return (hmc5883l_range_t)((i2c.readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void HMC5883L::setRange(hmc5883l_range_t range)
{
    switch(range)
    {
		case HMC5883L_RANGE_0_88GA:
			mgPerDigit = 0.073f;
			break;

		case HMC5883L_RANGE_1_3GA:
			mgPerDigit = 0.92f;
			break;

		case HMC5883L_RANGE_1_9GA:
			mgPerDigit = 1.22f;
			break;

		case HMC5883L_RANGE_2_5GA:
			mgPerDigit = 1.52f;
			break;

		case HMC5883L_RANGE_4GA:
			mgPerDigit = 2.27f;
			break;

		case HMC5883L_RANGE_4_7GA:
			mgPerDigit = 2.56f;
			break;

		case HMC5883L_RANGE_5_6GA:
			mgPerDigit = 3.03f;
			break;

		case HMC5883L_RANGE_8_1GA:
			mgPerDigit = 4.35f;
			break;

		default:
			break;
    }

    i2c.writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_mode_t HMC5883L::getMeasurementMode(void)
{
    uint8_t value;

    value = i2c.readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;
}

void HMC5883L::setMeasurementMode(hmc5883l_mode_t mode)
{
    uint8_t value;

    value = i2c.readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    i2c.writeRegister8(HMC5883L_REG_MODE, value);
}

hmc5883l_dataRate_t HMC5883L::getDataRate(void)
{
    uint8_t value;

    value = i2c.readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5883l_dataRate_t)value;
}

void HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = i2c.readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    i2c.writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t HMC5883L::getSamples(void)
{
    uint8_t value;

    value = i2c.readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5883l_samples_t)value;
}

void HMC5883L::setSamples(hmc5883l_samples_t samples)
{
    uint8_t value;

    value = i2c.readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    i2c.writeRegister8(HMC5883L_REG_CONFIG_A, value);
}