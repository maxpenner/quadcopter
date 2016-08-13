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
#include "mathHelp.h"
#include "voltageControl.h"

voltageControl::voltageControl()
{
	// all initializations go into setup function
}

voltageControl::~voltageControl()
{
}

void voltageControl::voltageControlSetup()
{
	pinMode(VOLTAGECONTROL_INPUT_PIN, INPUT);
	
	validValues = 0;
	movingAverageFilterLength = VOLTAGECONTROL_MOVAVG_LENGTH_MAX;
	
	// connect stack to ring buffer
	voltageValues = ringBuffer<unsigned int>(voltageValuesStack, VOLTAGECONTROL_MOVAVG_LENGTH_MAX);	
}

float voltageControl::getVoltage()
{
	unsigned int tmp = (unsigned int) analogRead(VOLTAGECONTROL_INPUT_PIN);
	
	voltageValues.pushNewElem(tmp);
	
	validValues = constrainn<int>(validValues + 1, 1, VOLTAGECONTROL_MOVAVG_LENGTH_MAX);
	
	int usable_values = (validValues < movingAverageFilterLength) ? validValues : movingAverageFilterLength;
	
	// calculate average voltage value
	unsigned long voltage = 0;
	for(int i=0; i<usable_values; i++)
	{
		unsigned int tmp1;
		voltageValues.getNthElem(tmp1, i);
		voltage += (unsigned long) tmp1;
	}	
	voltage /= usable_values;
	
	// tranlate from arduino
	float voltage_f = voltage/((float) ANALOG_READ_MAX);
	
	return voltage_f*VOLTAGECONTROL_SCALE;
}

void voltageControl::setMovAvgFilterLength(int mafl)
{
	movingAverageFilterLength = constrainn<int>(mafl, 1, VOLTAGECONTROL_MOVAVG_LENGTH_MAX);
}