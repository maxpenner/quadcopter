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

#ifndef QC_CONTROLLER_H
#define QC_CONTROLLER_H

#include "debug.h"
#include "mathHelp.h"
#include "vectorTypes.h"
#include "PID.h"

// measured on 07.07.2016
#define STABILIZER_HOVER_THRUST_PWM			1560.0f

#define STABILIZER_ROLL_PITCH_TOTAL_TILT_COMPENSATION
#define STABILIZER_BODY_RATES_CONVERSION
#define STABILIZER_THRUST_COMPENSATION

typedef enum
{
	STABILIZE = 0,
	STABILIZE_HEIGHT = 1
}stable_flight_mode;

class stabilizer
{
	public:
	
		stabilizer();
		~stabilizer();
		
		void setInitYawLock(float yawInitlock);
		void setInitHeightLock(float heightInitlock);
		void resetIntegrals();
		void setFlightMode(stable_flight_mode flight_mode_arg);
		Vector4f compute_pwmDutyCycle( 	Vector3f RPY_is,
										Vector3f RPYDot_is,
										float height_is,
										float heightDot_is,
										float heightDotDot_is,
										float roll_rx,
										float pitch_rx,
										float yaw_rx,
										float thrust_rx);
		
	private:
	
		stable_flight_mode flight_mode;

		// two stage PID cascade
		PID pidRoll[2];
		PID pidPitch[2];
		
		// first pid for locked yaw, second pid for yaw rotation
		PID pidYaw[2];
		float yawLock;

		// first pid for locked height
		PID pidHeight[2];
		float heightLock;	
		
		// transformation function
		Vector3f getBodyRatesFromEulerRates(Vector3f eulerAngles, Vector3f eulerRatesDesired);
		
		#ifdef DBG_STABLE_MAP_RX_INPUT_PRINT
		unsigned long lastMappedRxInputPrint;
		void printMappedRxInput(float roll, float pitch, float yaw, float thrust, float totalTilt);
		#endif
		
		#ifdef DBG_STABLE_SENSOR_INPUT_PRINT
		unsigned long lastSensorInputPrint;
		void printSensorInput(Vector3f RPY, Vector3f RPYDot, float height, float heightDot, float heightDotDot);
		#endif
		
		#ifdef DBG_STABLE_PWM_OUTPUT_PRINT
		unsigned long lastPWMOutputPrint;
		void printPWMOutput(float pwm0, float pwm1, float pwm2, float pwm3);
		#endif
		
		#ifdef DBG_STABLE_RANDOM_PRINT
		unsigned long lastStableRandomPrint;
		void printStableRandomData(Vector3f random0, Vector3f random1, Vector3f random2);
		#endif			
};

#endif