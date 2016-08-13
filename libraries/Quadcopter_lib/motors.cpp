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
#include <Arduino.h>
#include "motors.h"

motors::motors()
{
	// all initializations go into the setup function
}

motors::~motors()
{
}

void motors::motorsSetup()
{
	motorsReleased = false;
	motorsPanicStopped = false;
	
	arduinoMaxPulseLenght = 1000000.0f/MOTORS_ESC_PWM_FREQ;
	
	float tmp = mapp<float>(MOTORS_ESC_PWM_MIN, 0.0f, arduinoMaxPulseLenght, 0.0f, (float) MOTORS_12BIT_LIMIT);
	tmp = ceil(tmp);
	lowerClippingValue = (int) tmp;	
	
	// define pins as output
	pinMode(MOTORS_PIN_PWM0, OUTPUT);
	pinMode(MOTORS_PIN_PWM1, OUTPUT);
	pinMode(MOTORS_PIN_PWM2, OUTPUT);
	pinMode(MOTORS_PIN_PWM3, OUTPUT);
}

void motors::sendFullSignal()
{	
	analogWrite(MOTORS_PIN_PWM0, MOTORS_12BIT_LIMIT);
	analogWrite(MOTORS_PIN_PWM1, MOTORS_12BIT_LIMIT);
	analogWrite(MOTORS_PIN_PWM2, MOTORS_12BIT_LIMIT);
	analogWrite(MOTORS_PIN_PWM3, MOTORS_12BIT_LIMIT);
	
	#ifdef DBG_MOTORS_PWM_PRINT
	printPWMData(MOTORS_12BIT_LIMIT, MOTORS_12BIT_LIMIT, MOTORS_12BIT_LIMIT, MOTORS_12BIT_LIMIT);
	#endif	
}

void motors::sendZeroSignal()
{	
	analogWrite(MOTORS_PIN_PWM0, 0);
	analogWrite(MOTORS_PIN_PWM1, 0);
	analogWrite(MOTORS_PIN_PWM2, 0);
	analogWrite(MOTORS_PIN_PWM3, 0);
	
	#ifdef DBG_MOTORS_PWM_PRINT
	printPWMData(0.0f, 0.0f, 0.0f, 0.0f);
	#endif	
}

void motors::sendMinSignal()
{	
	convert2MotorSignal(MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MIN);
}

void motors::release()
{	
	motorsReleased = true;
}

void motors::block()
{	
	motorsReleased = false;
}

void motors::panicStop()
{
	sendMinSignal();
	motorsPanicStopped = true;
}
	
void motors::feedStabilizerPWM(const Vector4f pwmDutyCycle)
{
	if(motorsReleased == false || motorsPanicStopped == true)
	{
		sendMinSignal();
		return;
	}
	
	convert2MotorSignal(pwmDutyCycle.x, pwmDutyCycle.y , pwmDutyCycle.z, pwmDutyCycle.q);
}

void motors::convert2MotorSignal(float pwm0, float pwm1, float pwm2, float pwm3)
{
	// clip input pulse widths
	float motor0 = constrainn<float>(pwm0, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MAX);
	float motor1 = constrainn<float>(pwm1, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MAX);
	float motor2 = constrainn<float>(pwm2, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MAX);
	float motor3 = constrainn<float>(pwm3, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MAX);
	
	#ifdef DBG_MOTORS_PWM_PRINT
	printPWMData(pwm0, pwm1, pwm2, pwm3);
	#endif		
	
	// map to maximum range with 12 Bit resolution
	motor0 = mapp<float>(motor0, 0.0f, arduinoMaxPulseLenght, 0.0f, (float) MOTORS_12BIT_LIMIT);
	motor1 = mapp<float>(motor1, 0.0f, arduinoMaxPulseLenght, 0.0f, (float) MOTORS_12BIT_LIMIT);
	motor2 = mapp<float>(motor2, 0.0f, arduinoMaxPulseLenght, 0.0f, (float) MOTORS_12BIT_LIMIT);
	motor3 = mapp<float>(motor3, 0.0f, arduinoMaxPulseLenght, 0.0f, (float) MOTORS_12BIT_LIMIT);
	
	// convert float to integer values
	int motor0i = (int) motor0;
	int motor1i = (int) motor1;
	int motor2i = (int) motor2;
	int motor3i = (int) motor3;
	
	// clip integers
	motor0i = constrainn<int>(motor0i, lowerClippingValue, MOTORS_12BIT_LIMIT);
	motor1i = constrainn<int>(motor1i, lowerClippingValue, MOTORS_12BIT_LIMIT);
	motor2i = constrainn<int>(motor2i, lowerClippingValue, MOTORS_12BIT_LIMIT);
	motor3i = constrainn<int>(motor3i, lowerClippingValue, MOTORS_12BIT_LIMIT);
	
	// set new motor output
	analogWrite(MOTORS_PIN_PWM0, motor0i);
	analogWrite(MOTORS_PIN_PWM1, motor1i);
	analogWrite(MOTORS_PIN_PWM2, motor2i);
	analogWrite(MOTORS_PIN_PWM3, motor3i);	
}

#ifdef DBG_MOTORS_PWM_PRINT
void motors::printPWMData(float m0, float m1, float m2, float m3)
{
	Serial.print("\nM0 = "); Serial.print(m0); 
	Serial.print(" M1 = "); Serial.print(m1); 
	Serial.print(" M2 = "); Serial.print(m2);
	Serial.print(" M3 = "); Serial.println(m3);
}
#endif	