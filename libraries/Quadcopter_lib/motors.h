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
#ifndef QC_MOTORS_H
#define QC_MOTORS_H

#include "global.h"
#include "debug.h"
#include "mathHelp.h"
#include "vectorTypes.h"

/* These PWM pins are the due's hardware PWM pins (has 8 PWM channels that not depend on timers), so it's best to keep them.
 * Datasheet pins: PC24, PC23, PC22, PC21 (page 974).
 * Changes for 400 Hz PWM made in: C:\Users\<username>\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.6\variants\arduino_due_x\variant.h
 */
#define MOTORS_PIN_PWM0		6			// front left m0
#define MOTORS_PIN_PWM1		7			// front right m1
#define MOTORS_PIN_PWM2		8			// back right m2
#define MOTORS_PIN_PWM3		9			// back left m3

#define MOTORS_ESC_PWM_FREQ 400.0f		// pulses per second and motor

// Global interface to motors speeds from stillstand to full speed.
// Goes from 1000 to 2000. This should not be changed and is compatible to the receiver.
#define MOTORS_ESC_PWM_MIN	1000.0f		// absolute stillstand
#define MOTORS_ESC_PWM_STA	1100.0f		// minimum spinning speed
#define MOTORS_ESC_PWM_MAX	2000.0f		// full speed

#define MOTORS_12BIT_LIMIT	PWM_RESOLUTION_MAX

class motors
{
	public:

		motors();
		~motors();
		
		// init arduino related stuff in setup
		void motorsSetup();
		
		// motor access functions
		void sendZeroSignal();
		void sendFullSignal();
		void sendMinSignal();
		void release();
		void block();
		void panicStop();
		
		void feedStabilizerPWM(const Vector4f pwmDutyCycle);
		
	private:
	
		bool motorsReleased;
		bool motorsPanicStopped;
		float arduinoMaxPulseLenght;	// we want pulses from MOTORS_ESC_PWM_MIN to MOTORS_ESC_PWM_MAX, but arduino gives from 0 to 1/MOTORS_ESC_PWM_FREQ
		int lowerClippingValue;			// shortest pulse length mapped to 12 Bit range
		
		void convert2MotorSignal(float motor0, float motor1, float motor2, float motor3);
		
		#ifdef DBG_MOTORS_PWM_PRINT
		void printPWMData(float m0, float m1, float m2, float m3);
		#endif	
};

#endif