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
#include "global.h"
#include "PID.h"

PID::PID()
{
	dT = GLB_LOOP_TIME/1000000.0f;
	
	setGains(0.0f, 0.0f, 0.0f);
	
	zeroErrorIntegral();
	setiTermLimit(0.0f);
	
	lastError = 0.0f;	
}

PID::~PID()
{
}

void PID::setGains(float kp_arg, float ki_arg, float kd_arg)
{
	kp = kp_arg;
	ki = ki_arg;
	kd = kd_arg;
}

void PID::setiTermLimit(float iTermLimit_arg)
{
	iTermLimit = fabs(iTermLimit_arg);
}

void PID::zeroErrorIntegral()
{
	errorIntegral = 0.0f;
}

float PID::compute(float target, float is)
{
	// error, integral of error and derivative of error
	float error = target - is;
	errorIntegral += error*dT;
	float errorDerivative = (error - lastError)/dT;
	
	// PID values
	float pTerm = kp*error;
	float iTerm = ki*errorIntegral;
	float dTerm = kd*errorDerivative;
	
	// limit iTerm
	iTerm = (iTerm > iTermLimit) ? iTermLimit : iTerm;
	iTerm = (iTerm <-iTermLimit) ?-iTermLimit : iTerm;
	
	// save error for next call
	lastError = error;
	
	return pTerm + iTerm + dTerm;
}

float PID::getErrorIntegral()
{
	return errorIntegral;
}