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

#include <Arduino.h>
#include "global.h"
#include "debug.h"
#include "scheduler.h"

scheduler::scheduler()
{
	// all initializations go into the setup function
}

scheduler::~scheduler()
{
}

void scheduler::schedulerSetup()
{
	programStartTime = micros();
	lastPeriodBegin = 0ul;
	doublePeriod = -1;
	triplePeriod = -1;
	quadruplePeriod = -1;
	decuplePeriod = -1;
	
	periodTimeMin = 4294967295ul;	// ULONG_MAX = 2^32-1
	periodTimeMax = 0ul;
	firstExecTime = 0ul;	
	netExecTimeMin = 4294967295ul;	// ULONG_MAX = 2^32-1
	netExecTimeMax = 0ul;
	#ifdef DBG_SCHEDULER_PIN
	pinMode(DBG_SCHEDULER_PIN, OUTPUT);
	pinState = true;
	digitalWrite(DBG_SCHEDULER_PIN, HIGH);
	#endif
	
	#ifdef DBG_SCHEDULER_PRINT
	lastExecTimePrint = programStartTime;
	#endif
}

void scheduler::setTime()
{
	lastPeriodBegin = micros();
	doublePeriod = -1;
	triplePeriod = -1;
	quadruplePeriod = -1;
	decuplePeriod = -1;
	
	periodTimeMin = 4294967295ul;
	periodTimeMax = 0ul;
	firstExecTime = 0ul;
	netExecTimeMin = 4294967295ul;
	netExecTimeMax = 0ul;
	
	#ifdef DBG_SCHEDULER_PRINT
	lastExecTimePrint = micros();
	#endif	
}

bool scheduler::checkBasePeriod()
{
	unsigned long now = micros();
	
	if(now - lastPeriodBegin >= GLB_LOOP_TIME)
	{
		unsigned long thisPeriodTime = now - lastPeriodBegin;
		lastPeriodBegin = now;
		
		// set counters for multiples of period time
		doublePeriod = ((doublePeriod+1)%2 == 0) ? 0 : doublePeriod+1;
		triplePeriod = ((triplePeriod+1)%3 == 0) ? 0 : triplePeriod+1;
		quadruplePeriod = ((quadruplePeriod+1)%4 == 0) ? 0 : quadruplePeriod+1;
		decuplePeriod = ((decuplePeriod+1)%10 == 0) ? 0 : decuplePeriod+1;
		
		// save min and max period time
		periodTimeMin = (thisPeriodTime < periodTimeMin) ? thisPeriodTime : periodTimeMin;
		periodTimeMax = (thisPeriodTime > periodTimeMax) ? thisPeriodTime : periodTimeMax;	
		
		#ifdef DBG_SCHEDULER_PIN
		// Reverse pin state so it can be measured with an oscilloscope. Frequency should be 1000000/(2*GLB_LOOP_TIME).
		// E.g. for GLB_LOOP_TIME = 10000 it should be 50 Hz.
		if(pinState == true)
		{
			pinState = false;
			digitalWrite(DBG_SCHEDULER_PIN, LOW);
		}
		else
		{
			pinState = true;
			digitalWrite(DBG_SCHEDULER_PIN, HIGH);
		}
		#endif		
		
		return true;
	}
	
	return false;
}


bool scheduler::checkDoublePeriod()
{
	return (doublePeriod == 0);
}

bool scheduler::checkTriplePeriod()
{
	return (triplePeriod == 0);
}

bool scheduler::checkQuadruplePeriod()
{
	return (quadruplePeriod == 0);
}

bool scheduler::checkDecuplePeriod()
{
	return (decuplePeriod == 0);
}

void scheduler::netExecTimeEnd()
{
	unsigned long now = micros();
	unsigned long execTime = now - lastPeriodBegin;
	
	// save min and max execution time
	netExecTimeMin = (execTime < netExecTimeMin) ? execTime : netExecTimeMin;
	if(firstExecTime == 0ul)
		firstExecTime = execTime;
	else
		netExecTimeMax = (execTime > netExecTimeMax) ? execTime : netExecTimeMax;
	
	#ifdef DBG_SCHEDULER_EXEC_TIME_PRINT
	DBG_println(execTime);
	#endif

	#ifdef DBG_SCHEDULER_PRINT
	if(now - lastExecTimePrint > DBG_SCHEDULER_PRINT)
	{
		if(periodTimeMax >= GLB_LOOP_TIME + DBG_SCHEDULER_OFFSET_PRINT)
		{
			lastExecTimePrint = now;
			Serial.print("\nTotal Exec Time:\t\t"); Serial.print((now - programStartTime)/1000000UL); Serial.println(" s");
			Serial.print("Min Period Time:\t\t"); Serial.print(periodTimeMin); Serial.println(" us");
			Serial.print("Max Period Time:\t\t"); Serial.print(periodTimeMax); Serial.println(" us");
			Serial.print("First Net Exec Time:\t\t"); Serial.print(firstExecTime); Serial.println(" us");
			Serial.print("Min Net Exec Time:\t\t"); Serial.print(netExecTimeMin); Serial.println(" us");
			Serial.print("Max Net Exec Time:\t\t"); Serial.print(netExecTimeMax); Serial.println(" us");
			Serial.print("Max Net Exec Time to Loop Time:\t"); Serial.print(netExecTimeMax*100UL/GLB_LOOP_TIME); Serial.println(" %");
			Serial.print("Printing Time:\t"); Serial.print(micros() - now); Serial.println(" us");			
		}		
	}
	#endif
}

unsigned long scheduler::getPeriodTimeMin()
{
	return periodTimeMin;
}

unsigned long scheduler::getPeriodTimeMax()
{
	return periodTimeMax;
}

unsigned long scheduler::getNetExecTimeMin()
{
	return netExecTimeMin;
}

unsigned long scheduler::getNetExecTimeMax()
{
	return netExecTimeMax;
}