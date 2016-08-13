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

// This library is a minimum library for HMC5883L. 
// It's only task it to retrieve the raw sensor data.
// source (with modifications): https://github.com/jarzebski

#ifndef QC_HMC5883L_H
#define QC_HMC5883L_H

#include "vectorTypes.h"
#include "I2C.h"

// ============================= CONFIGURATION ==============================

// Measured with CoolTerm and calibrated with magneto on 2016/03/11.
// Calibration made for total field strenght of 440uT (Germany).
#define HMC5883L_BIAS_X		-73.170060f
#define HMC5883L_BIAS_Y		37.848642f
#define HMC5883L_BIAS_Z		-197.116844f

#define HMC5883L_A11		0.819042f
#define HMC5883L_A12		0.006085f
#define HMC5883L_A13		0.022565f
#define HMC5883L_A21		0.006085f
#define HMC5883L_A22		0.827778f
#define HMC5883L_A23		-0.000869f
#define HMC5883L_A31		0.022565f
#define HMC5883L_A32		-0.000869f
#define HMC5883L_A33		0.858837f

// ============================ CONFIGURATION END ===========================

#define HMC5883L_ADDRESS              (0x1E)
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

typedef enum
{
    HMC5883L_SAMPLES_8	= 0b11,
    HMC5883L_SAMPLES_4	= 0b10,
    HMC5883L_SAMPLES_2	= 0b01,
    HMC5883L_SAMPLES_1	= 0b00
}hmc5883l_samples_t;

typedef enum
{
    HMC5883L_DATARATE_75HZ		= 0b110,
    HMC5883L_DATARATE_30HZ		= 0b101,
    HMC5883L_DATARATE_15HZ		= 0b100,
    HMC5883L_DATARATE_7_5HZ		= 0b011,
    HMC5883L_DATARATE_3HZ		= 0b010,
    HMC5883L_DATARATE_1_5HZ		= 0b001,
    HMC5883L_DATARATE_0_75_HZ	= 0b000
}hmc5883l_dataRate_t;

typedef enum
{
    HMC5883L_RANGE_8_1GA	= 0b111,
    HMC5883L_RANGE_5_6GA	= 0b110,
    HMC5883L_RANGE_4_7GA	= 0b101,
    HMC5883L_RANGE_4GA		= 0b100,
    HMC5883L_RANGE_2_5GA	= 0b011,
    HMC5883L_RANGE_1_9GA	= 0b010,
    HMC5883L_RANGE_1_3GA	= 0b001,
    HMC5883L_RANGE_0_88GA	= 0b000
}hmc5883l_range_t;

typedef enum
{
    HMC5883L_IDLE		= 0b10,
    HMC5883L_SINGLE		= 0b01,
    HMC5883L_CONTINUOUS	= 0b00
}hmc5883l_mode_t;

class HMC5883L
{
    public:
	
		bool begin(void);

		Vector3f readRaw(void);			// register integer values converted to float
		Vector3f readNormalize(void);	// milli gauss
		Vector3f readCalibrated(void);	// milli gauss

		hmc5883l_range_t getRange(void);
		void setRange(hmc5883l_range_t range);

		hmc5883l_mode_t getMeasurementMode(void);
		void setMeasurementMode(hmc5883l_mode_t mode);

		hmc5883l_dataRate_t getDataRate(void);
		void setDataRate(hmc5883l_dataRate_t dataRate);

		hmc5883l_samples_t getSamples(void);
		void setSamples(hmc5883l_samples_t samples);

    private:
	
		float mgPerDigit;

		I2C i2c;
};

#endif