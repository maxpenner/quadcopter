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

#include <math.h>
#include "mathHelp.h"
#include "receiver.h"
#include "motors.h"
#include "stabilizer.h"

#if defined(DBG_STABLE_MAP_RX_INPUT_PRINT) || defined(DBG_STABLE_SENSOR_INPUT_PRINT) || defined(DBG_STABLE_PWM_OUTPUT_PRINT) || defined(DBG_STABLE_RANDOM_PRINT)
#include <Arduino.h>
#endif

#define STAB 0
#define RATE 1

#define HEIGHT_M 0
#define HEIGHTDOT_M_S 1

stabilizer::stabilizer()
{
	flight_mode = STABILIZE;
	
	// roll and pitch
	pidRoll[STAB].setGains(4.5f, 0.0f, 0.0f);
	pidRoll[RATE].setGains(0.9f, 1.0f, 0.0f);
	pidRoll[RATE].setiTermLimit(50.0f);
	pidPitch[STAB].setGains(4.5f, 0.0f, 0.0f);
	pidPitch[RATE].setGains(0.9, 1.0f, 0.0f);
	pidPitch[RATE].setiTermLimit(50.0f);
	
	// yaw
	pidYaw[STAB].setGains(1.5f, 0.0f, 0.0f);
	pidYaw[RATE].setGains(2.7f, 0.0f, 0.0f);
	pidYaw[RATE].setiTermLimit(0.0f);
	yawLock = 0.0f;

	// height
	pidHeight[HEIGHT_M].setGains(50.0f, 0.25f, 0.0f);
	pidHeight[HEIGHT_M].setiTermLimit(40.0f);
	pidHeight[HEIGHTDOT_M_S].setGains(200.0f, 1.0f, 0.0f);
	pidHeight[HEIGHTDOT_M_S].setiTermLimit(100.0f);
	heightLock = 0.0f;	
	
	#ifdef DBG_STABLE_MAP_RX_INPUT_PRINT
	lastMappedRxInputPrint = 0;
	#endif
	
	#ifdef DBG_STABLE_SENSOR_INPUT_PRINT
	lastSensorInputPrint = 0;
	#endif

	#ifdef DBG_STABLE_PWM_OUTPUT_PRINT
	lastPWMOutputPrint = 0;
	#endif
	
	#ifdef DBG_STABLE_RANDOM_PRINT
	lastStableRandomPrint = 0;
	#endif	
}

stabilizer::~stabilizer()
{	
}

void stabilizer::setInitYawLock(float yawInitlock)
{
	// height cannot be locked at the beginning -> user must start in STABILIZE mode
	yawLock = yawInitlock*RAD2DEG;
}

void stabilizer::setInitHeightLock(float heightInitlock)
{
	heightLock = heightInitlock;
}

void stabilizer::resetIntegrals()
{
	pidRoll[STAB].zeroErrorIntegral();
	pidRoll[RATE].zeroErrorIntegral();
	pidPitch[STAB].zeroErrorIntegral();
	pidPitch[RATE].zeroErrorIntegral();
	pidYaw[STAB].zeroErrorIntegral();
	pidYaw[RATE].zeroErrorIntegral();
	pidHeight[HEIGHT_M].zeroErrorIntegral();
	pidHeight[HEIGHTDOT_M_S].zeroErrorIntegral();
}

void stabilizer::setFlightMode(stable_flight_mode flight_mode_arg)
{
	flight_mode = flight_mode_arg;
}

Vector4f stabilizer::compute_pwmDutyCycle( 	Vector3f RPY_is,
											Vector3f RPYDot_is,
											float height_is,
											float heightDot_is,
											float heightDotDot_is,
											float roll_rx,
											float pitch_rx,
											float yaw_rx,
											float thrust_rx)
{	
	// map rx input to target angles
	roll_rx = mapp<float>(roll_rx, RECV_CLIP_MINIMUM, RECV_CLIP_MAXIMUM, -30.0f, 30.0f);
	pitch_rx = mapp<float>(pitch_rx, RECV_CLIP_MINIMUM, RECV_CLIP_MAXIMUM, -30.0f, 30.0f);
	yaw_rx = mapp<float>(yaw_rx, RECV_CLIP_MINIMUM, RECV_CLIP_MAXIMUM, -135.0f, 135.0f);
	yaw_rx = -yaw_rx;
	
	// receiver is not perfect, create dead band
	roll_rx = deadband<float>(roll_rx, 0.75f);
	pitch_rx = deadband<float>(pitch_rx, 0.75f);
	
	#ifdef STABILIZER_ROLL_PITCH_TOTAL_TILT_COMPENSATION
	double totalTilt = RAD2DEG*acos(cos(DEG2RAD*roll_rx)*cos(DEG2RAD*pitch_rx));
	if(totalTilt > 30.0f)
	{
		roll_rx *= 30.0f/totalTilt;
		pitch_rx *= 30.0f/totalTilt;
	}
	#endif
	
	#ifdef DBG_STABLE_MAP_RX_INPUT_PRINT
	printMappedRxInput(roll_rx, pitch_rx, yaw_rx, thrust_rx, RAD2DEG*acos(cos(DEG2RAD*roll_rx)*cos(DEG2RAD*pitch_rx)));
	#endif
	
	// convert fused angles to degrees
	Vector3f RPY_is_deg = RPY_is;
	Vector3f RPYDot_is_deg = RPYDot_is;
	RPY_is_deg *= RAD2DEG;
	RPYDot_is_deg *= RAD2DEG;
	
	#ifdef DBG_STABLE_SENSOR_INPUT_PRINT
	printSensorInput(RPY_is_deg, RPYDot_is_deg, height_is, heightDot_is, heightDotDot_is);
	#endif
	
	// values to be determined
	float roll_out = 0.0f, pitch_out = 0.0f, yaw_out = 0.0f, thrust_out = 0.0f;	

	// first angle control stage
	float roll_stab_output = constrainn<float>(pidRoll[STAB].compute(roll_rx, RPY_is_deg.x), -250.0f, 250.0f);
	float pitch_stab_output = constrainn<float>(pidPitch[STAB].compute(pitch_rx, RPY_is_deg.y), -250.0f, 250.0f);
	float yaw_error = wrap_180(yawLock - RPY_is_deg.z);
	float yaw_stab_output = constrainn<float>(pidYaw[STAB].compute(yaw_error, 0.0), -360.0f, 360.0f);

	// if pilot is asking for yaw change feed signal directly to rate pid
	if(fabs(deadband<float>(yaw_rx, 5.0f)) > 0.0f)
	{
		yaw_stab_output = yaw_rx;
		yawLock = RPY_is_deg.z;
	}
	
	#ifdef STABILIZER_BODY_RATES_CONVERSION
	// is rates (changing meaning of RPYDot_is!)
	Vector3f RPYDot_is_deg_body = getBodyRatesFromEulerRates(RPY_is, RPYDot_is);
	RPYDot_is_deg_body *= RAD2DEG;
	
	// target rates
	Vector3f target_body_rates = getBodyRatesFromEulerRates(RPY_is, Vector3f(DEG2RAD*roll_stab_output, DEG2RAD*pitch_stab_output, DEG2RAD*yaw_stab_output));
	roll_stab_output = target_body_rates.x*RAD2DEG;
	pitch_stab_output = target_body_rates.y*RAD2DEG;
	yaw_stab_output = target_body_rates.z*RAD2DEG;
		
    // second angle control stage
	roll_out = constrainn<float>(pidRoll[RATE].compute(roll_stab_output, RPYDot_is_deg_body.x), - 500.0f, 500.0f);  
	pitch_out = constrainn<float>(pidPitch[RATE].compute(pitch_stab_output, RPYDot_is_deg_body.y), -500.0f, 500.0f);
	yaw_out = constrainn<float>(pidYaw[RATE].compute(yaw_stab_output, RPYDot_is_deg_body.z), -500.0f, 500.0f);
	#else
	roll_out = constrainn<float>(pidRoll[RATE].compute(roll_stab_output, RPYDot_is_deg.x), - 500.0f, 500.0f);  
	pitch_out = constrainn<float>(pidPitch[RATE].compute(pitch_stab_output, RPYDot_is_deg.y), -500.0f, 500.0f);
	yaw_out = constrainn<float>(pidYaw[RATE].compute(yaw_stab_output, RPYDot_is_deg.z), -500.0f, 500.0f);
	#endif
	
	// height stabilization
	if(flight_mode == STABILIZE)
	{
		thrust_out = mapp<float>(thrust_rx, RECV_CLIP_MINIMUM, RECV_CLIP_MAXIMUM, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MAX);
		#ifdef STABILIZER_THRUST_COMPENSATION
		thrust_out /= constrainn<float>(cos(RPY_is.x)*cos(RPY_is.y), 0.5f, 1.0f);
		#endif
		thrust_out = constrainn<float>(thrust_out, 0.0f, MOTORS_ESC_PWM_MAX - 0.0f);
		
		// save height in case mode changes
		heightLock = height_is;
	}
	else if(flight_mode == STABILIZE_HEIGHT)
	{
		thrust_out = STABILIZER_HOVER_THRUST_PWM;
		
		// map thrust from rx to zero centered signal (throttle stick must be moved into hover position)
		thrust_rx -= STABILIZER_HOVER_THRUST_PWM;
		
		float outOfDeadbandSignal = deadband<float>(thrust_rx, 100.0f);
		
		// if pilot is asking for height change
		if(fabs(outOfDeadbandSignal) > 0.0f)
		{
			thrust_out += outOfDeadbandSignal;
			heightLock = height_is;
			pidHeight[HEIGHT_M].zeroErrorIntegral();
			pidHeight[HEIGHTDOT_M_S].zeroErrorIntegral();
		}
		else
		{
			// height controlled only in discrete levels
			float height_step = 1.0f;
			int height_level_difference = (int) ((height_is - heightLock)/height_step);
			float height_discrete = heightLock + ((float) height_level_difference)*height_step;

			// first stage; controll height difference
			float additional_thrust = constrainn<float>(pidHeight[HEIGHT_M].compute(heightLock, height_discrete), -25.0f, 25.0f);

			// second state: controll climb/sink rate
			//additional_thrust = pidHeight[HEIGHTDOT_M_S].compute(additional_thrust, heightDot_is);

			thrust_out += additional_thrust;
		}

		#ifdef STABILIZER_THRUST_COMPENSATION
		thrust_out /= constrainn<float>(cos(RPY_is.x)*cos(RPY_is.y), 0.5f, 1.0f);
		#endif
		thrust_out = constrainn<float>(thrust_out, STABILIZER_HOVER_THRUST_PWM - 350.0f, STABILIZER_HOVER_THRUST_PWM + 350.0f);
	}
	
	// translate into pwm signal (x mode)
	float pwm0 = thrust_out + roll_out - pitch_out + yaw_out;
	float pwm1 = thrust_out - roll_out - pitch_out - yaw_out;
	float pwm2 = thrust_out - roll_out + pitch_out + yaw_out;
	float pwm3 = thrust_out + roll_out + pitch_out - yaw_out;
	
	// during flight motors cannot be turned off completely
	pwm0 = constrainn<float>(pwm0, MOTORS_ESC_PWM_STA, MOTORS_ESC_PWM_MAX);
	pwm1 = constrainn<float>(pwm1, MOTORS_ESC_PWM_STA, MOTORS_ESC_PWM_MAX);
	pwm2 = constrainn<float>(pwm2, MOTORS_ESC_PWM_STA, MOTORS_ESC_PWM_MAX);
	pwm3 = constrainn<float>(pwm3, MOTORS_ESC_PWM_STA, MOTORS_ESC_PWM_MAX);	
	
	#ifdef DBG_STABLE_PWM_OUTPUT_PRINT
	printPWMOutput(pwm0, pwm1, pwm2, pwm3);
	#endif
	
	#ifdef DBG_STABLE_RANDOM_PRINT
	printStableRandomData(Vector3f(height_is, heightLock, 0.0f), Vector3f(dbg0, dbg1, 0.0f), Vector3f(0.0f, 0.0f, 0.0f));
	#endif	
	
	return Vector4f(pwm0, pwm1, pwm2, pwm3);
}

Vector3f stabilizer::getBodyRatesFromEulerRates(Vector3f eulerAngles, Vector3f eulerRatesDesired)
{
	// Source: http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
	float sin_phi = sin(eulerAngles.x);
	float cos_phi = cos(eulerAngles.x);
	float sin_theta = sin(eulerAngles.y);
	float cos_theta = cos(eulerAngles.y);
	
	Vector3f BodyRates;

	BodyRates.x = eulerRatesDesired.x                               -         sin_theta*eulerRatesDesired.z;
	BodyRates.y =                       cos_phi*eulerRatesDesired.y + sin_phi*cos_theta*eulerRatesDesired.z;
	BodyRates.z =                     - sin_phi*eulerRatesDesired.y + cos_phi*cos_theta*eulerRatesDesired.z;
	
	return BodyRates;
}

#ifdef DBG_STABLE_MAP_RX_INPUT_PRINT
void stabilizer::printMappedRxInput(float roll, float pitch, float yaw, float thrust, float totalTilt)
{
	unsigned long now = micros();
	
	if(now - lastMappedRxInputPrint >= DBG_STABLE_MAP_RX_INPUT_PRINT)
	{
		lastMappedRxInputPrint = now;
		
		float mapped2Motor = mapp<float>(thrust, RECV_CLIP_MINIMUM, RECV_CLIP_MAXIMUM, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MAX);
		
		Serial.print("\nR_rx = "); Serial.print(roll); Serial.print(" P_rx = "); Serial.print(pitch); Serial.print(" Y_rx = "); Serial.println(yaw);
		Serial.print("TotalTilt = "); Serial.println(totalTilt);
		Serial.print("Thrust mapped to Motor = "); Serial.println(mapped2Motor);
	}
}
#endif

#ifdef DBG_STABLE_SENSOR_INPUT_PRINT
void stabilizer::printSensorInput(Vector3f RPY, Vector3f RPYDot, float height, float heightDot, float heightDotDot)
{
	unsigned long now = micros();
	
	if(now - lastSensorInputPrint >= DBG_STABLE_SENSOR_INPUT_PRINT)
	{
		lastSensorInputPrint = now;
		
		Serial.print("\nRoll = "); Serial.print(RPY.x); Serial.print(" Pitch = "); Serial.print(RPY.y); Serial.print(" Yaw = "); Serial.println(RPY.z);
		Serial.print("RollDot = "); Serial.print(RPYDot.x); Serial.print(" PitchDot = "); Serial.print(RPYDot.y); Serial.print(" YawDot = "); Serial.println(RPYDot.z);
		Serial.print("Height = "); Serial.print(height); Serial.print(" HeightDot = "); Serial.print(heightDot); Serial.print(" HeightDotDot = "); Serial.println(heightDotDot);
	}
}
#endif

#ifdef DBG_STABLE_PWM_OUTPUT_PRINT
void stabilizer::printPWMOutput(float pwm0, float pwm1, float pwm2, float pwm3)
{
	unsigned long now = micros();
	
	if(now - lastPWMOutputPrint >= DBG_STABLE_PWM_OUTPUT_PRINT)
	{
		lastPWMOutputPrint = now;
		
		Serial.print("\nPWM0 = "); Serial.print(pwm0); Serial.print(" PWM1 = "); Serial.print(pwm1); 
		Serial.print(" PWM2 = "); Serial.print(pwm2); Serial.print(" PWM3 = "); Serial.println(pwm3);
	}
}
#endif

#ifdef DBG_STABLE_RANDOM_PRINT
void stabilizer::printStableRandomData(Vector3f random0, Vector3f random1, Vector3f random2)
{
	unsigned long now = micros();
	
	if(now - lastStableRandomPrint >= DBG_STABLE_RANDOM_PRINT)
	{
		lastStableRandomPrint = now;
		
		Serial.print("\nR0 x = "); Serial.print(random0.x); Serial.print(" y = "); Serial.print(random0.y); Serial.print(" z = "); Serial.println(random0.z);
		Serial.print("R1 x = "); Serial.print(random1.x); Serial.print(" y = "); Serial.print(random1.y); Serial.print(" z = "); Serial.println(random1.z);
		Serial.print("R2 x = "); Serial.print(random2.x); Serial.print(" y = "); Serial.print(random2.y); Serial.print(" z = "); Serial.println(random2.z);
	}
}
#endif