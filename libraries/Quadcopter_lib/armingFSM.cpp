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
#include "armingFSM.h"

armingFSM::armingFSM()
{
	armed = false;
	armingProcessState = WAITING;
}

armingFSM::~armingFSM()
{
}

bool armingFSM::refreshArmingState(float thrust_rx, float yaw_rx)
{
	// after successfull arming/disarming yaw has to be moved to center position
	if(armingProcessState == BLOCKING)
	{
		if(checkLimitValue(yaw_rx, 1500.0f, 25.0f))
		{
			armingProcessState = WAITING;
		}
		else
		{
			return armed;
		}
	}
	
	// thrust in lowest position?
	if(checkLimitValue(thrust_rx, 1000.0f, 25.0f))
	{
		// reset timer if no arming/disarming process has been started
		if(armingProcessState == WAITING)
			armingStart = micros();
		
		// check if user wants to arm or disarm
		if(checkLimitValue(yaw_rx, 1000.0f, 25.0f))
		{
			armingProcessState = ARMING;
		}
		else if(checkLimitValue(yaw_rx, 2000.0f, 25.0f))
		{
			armingProcessState = DISARMING;
		}
		else
		{
			armingProcessState = WAITING;
		}
		
		switch(armingProcessState)
		{
			case WAITING:
				break;
				
			case ARMING:
				if(micros() - armingStart >= ARMINGFMS_ARMING_SIGNAL_LENGTH*1000)
				{
					armed = true;
					armingProcessState = BLOCKING;
				}
				break;
				
			case DISARMING:
				if(micros() - armingStart >= ARMINGFMS_DISARMING_SIGNAL_LENGTH*1000)
				{
					armed = false;
					armingProcessState = BLOCKING;
				}
				break;
		}
	}
	else
	{		
		armingProcessState = WAITING;	
	}
	
	return armed;	
}

bool armingFSM::getArmingState()
{
	return armed;
}

armingFSM_process armingFSM::getArmingProcessState()
{
	return armingProcessState;
}

bool armingFSM::checkLimitValue(float value, float center, float limits)
{
	float distance = fabs(value - center);
	
	return (distance <= limits);
}