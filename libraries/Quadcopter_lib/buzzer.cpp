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
#include "buzzer.h"

#include "debug.h"

buzzer::buzzer()
{
	// all initializations go into the setup function
}

buzzer::~buzzer()
{
}

void buzzer::buzzerSetup()
{
	continuousSoundSubscribed = false;
	melodySubscribed = false;
	
	// start zero output
	pinMode(BUZZER_OUTPUT_PIN, OUTPUT);
	buzzStop();
}

void buzzer::startContinuousSound()
{
	melodySubscribed = false;
	continuousSoundSubscribed = true;
	buzz();
}

void buzzer::stopContinuousSound()
{
	if(continuousSoundSubscribed == true)
	{
		buzzStop();
		continuousSoundSubscribed = false;
	}
}

bool buzzer::startMelody(unsigned long _pulses, unsigned long _pulsesLength, unsigned long _finalPause, bool _blockingCall)
{
	// ignore if busy
	if((melodySubscribed == true) || (continuousSoundSubscribed == true))
		return false;

	// from now on busy
	melodySubscribed = true;
	
	// limit desired pulse formation
	pulses = constrainn<unsigned long>(_pulses, 1, BUZZER_PULSES_MAX);
	pulsesLength = 1000*constrainn<unsigned long>(_pulsesLength, BUZZER_PULSES_LENGTH_MIN, BUZZER_PULSES_LENGTH_MAX);
	finalPause = 1000*constrainn<unsigned long>(_finalPause, 0, BUZZER_FINAL_PAUSE_LENGTH_MAX);
	blockingCall = _blockingCall;
	
	// reset state machine
	melodySubState = 0;
	melodySubStateStart = micros();
	startFinalPause = false;
	
	// start making sound
	buzz();
	update();
	
	return true;
}

void buzzer::stopMelody()
{
	if(melodySubscribed == true)
	{
		buzzStop();
		melodySubscribed = false;
	}
}

void buzzer::update()
{	
	// check if anything to update
	if(melodySubscribed == false)
		return;
		
	if(blockingCall)
	{
		while(!pulse_x_fsm(pulses, pulsesLength, finalPause))
		{
			delay(50);
		}
		melodySubscribed = false;
	}
	else
	{
		if(pulse_x_fsm(pulses, pulsesLength, finalPause))
			melodySubscribed = false;
	}
}

void buzzer::buzz()
{
	unsigned long tmp = PWM_RESOLUTION_MAX*BUZZER_OUTPUT_DUTYCYCLE/100;
	
	analogWrite(BUZZER_OUTPUT_PIN, tmp);	
}

void buzzer::buzzStop()
{
	analogWrite(BUZZER_OUTPUT_PIN, 0);	
}

bool buzzer::pulse_x_fsm(unsigned long pulses, unsigned long pulseLength, unsigned long finalPauseLength)
{
	bool fsm_done = false;
	unsigned long now = micros();
	
	if(!startFinalPause)
	{
		switch(melodySubState % 2)
		{
			case 0:
				if(now - melodySubStateStart >= pulseLength)
				{
					buzzStop();
					melodySubStateStart = now;
					melodySubState++;
				}
				break;
				
			case 1:
				if(now - melodySubStateStart >= BUZZER_STD_PAUSE*1000)
				{
					int played_pulses = (melodySubState+1)/2;
					
					if(played_pulses == pulses)
					{
						if(finalPauseLength == 0)
						{
							fsm_done = true;
						}
						else
						{
							startFinalPause = true;
							melodySubStateStart = now;
						}
					}
					else
					{
						buzz();
						melodySubState++;
						melodySubStateStart = now;
					}
				}
				break;	
		}		
	}
	else
	{
		if(now - melodySubStateStart >= finalPauseLength)
			fsm_done = true;
	}
	
	return fsm_done;
}