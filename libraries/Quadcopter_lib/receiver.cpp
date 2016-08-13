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
#include "mathHelp.h"
#include "receiver.h"

#ifdef DBG_RECEIVER_PRINT
static unsigned int lastRawPrint;
#endif

static volatile unsigned int pulseLength[RECV_N_CHANNEL];
static volatile unsigned int timeLastChange[RECV_N_CHANNEL];

static volatile unsigned int pulseLengthOutsideLimitsN;
static volatile unsigned int pulseLengthOutsideLimitsValue;

// interrupt handlers
static void ch0Handler();
static void ch1Handler();
static void ch2Handler();
static void ch3Handler();
static void ch4Handler();
static void ch5Handler();
static void pwmHandler(int chann, int pin);

static unsigned int getRawChannelValue(enum recv_channel index);

namespace receiverModule
{
	void init()
	{
		for(int i=0; i<RECV_N_CHANNEL; i++)
		{
			pulseLength[i] = 0;
			timeLastChange[i] = 0;
		}
		
		resetLimitStats();
		
		// tell the Power Management Controller to disable the write protection of the (Timer/Counter) registers
		pmc_set_writeprotect(false);

		// enable external clock for counter TC2, channel 2
		pmc_enable_periph_clk(ID_TC8);

		// set clock tp 84MHz/2 = 42MHz
		TC_Configure(TC2, 2, TC_CMR_TCCLKS_TIMER_CLOCK1);

		// start timer
		TC_Start(TC2,2);
		
		// connect pins and handlers
		attachInterrupt(RECV_CHAN0PIN, &ch0Handler, CHANGE);
		attachInterrupt(RECV_CHAN1PIN, &ch1Handler, CHANGE);
		attachInterrupt(RECV_CHAN2PIN, &ch2Handler, CHANGE);
		attachInterrupt(RECV_CHAN3PIN, &ch3Handler, CHANGE);
		attachInterrupt(RECV_CHAN4PIN, &ch4Handler, CHANGE);
		attachInterrupt(RECV_CHAN5PIN, &ch5Handler, CHANGE);
		
		#if defined(DBG_RECEIVER_PRINT)
		lastRawPrint = 0;
		#endif
	}
	
	void resetLimitStats()
	{
		pulseLengthOutsideLimitsN = 0;
		pulseLengthOutsideLimitsValue = 0;		
	}
	
	bool allChannelsInLimits()
	{
		if((getRawChannelValue(AILERON) <= RECV_PULSE_LENGTH_MIN) || (getRawChannelValue(AILERON) >= RECV_PULSE_LENGTH_MAX))
			return false;
			
		if((getRawChannelValue(ELEVATOR) <= RECV_PULSE_LENGTH_MIN) || (getRawChannelValue(ELEVATOR) >= RECV_PULSE_LENGTH_MAX))
			return false;
			
		if((getRawChannelValue(THROTTLE) <= RECV_PULSE_LENGTH_MIN) || (getRawChannelValue(THROTTLE) >= RECV_PULSE_LENGTH_MAX))
			return false;
			
		if((getRawChannelValue(RUDDER) <= RECV_PULSE_LENGTH_MIN) || (getRawChannelValue(RUDDER) >= RECV_PULSE_LENGTH_MAX))
			return false;
			
		if((getRawChannelValue(AUX0) <= RECV_PULSE_LENGTH_MIN) || (getRawChannelValue(AUX0) >= RECV_PULSE_LENGTH_MAX))
			return false;
			
		if((getRawChannelValue(AUX1) <= RECV_PULSE_LENGTH_MIN) || (getRawChannelValue(AUX1) >= RECV_PULSE_LENGTH_MAX))
			return false;															
		
		return true;
	}
	
	float getClippedChannelValue_f(enum recv_channel index)
	{
		unsigned int tmp = getRawChannelValue(index);
	
		return constrainn<float>((float) tmp, RECV_CLIP_MINIMUM, RECV_CLIP_MAXIMUM);
	};
	
	int checkZoneClippedValue(float channelValue, int n_zones)
	{
		if(n_zones == 1)
			return 0;
		
		float singleZoneLength = (RECV_CLIP_MAXIMUM - RECV_CLIP_MINIMUM) / ((float) (n_zones-1));
		
		// test all zones
		float min_distance = fabs(RECV_CLIP_MINIMUM - channelValue);
		int zone = 0;
		for(int i=1; i<n_zones; i++)
		{
			float distance = fabs(RECV_CLIP_MINIMUM + ((float) i)*singleZoneLength - channelValue);
			if(distance < min_distance)
			{
				min_distance = distance;
				zone = i;
			}
		}
		
		return zone;
	}
	
	bool checkLimitClippedValue(float channelValue, float center, float limits)
	{
		float distance = fabs(channelValue - center);
		
		return (distance <= limits);
	}
	
	#if defined(DBG_RECEIVER_PRINT)
	void printDebug()
	{
		unsigned int now = micros();
		
		if(now - lastRawPrint >= DBG_RECEIVER_PRINT)
		{
			lastRawPrint = now;  
			
			Serial.print("\nCH1: "); Serial.print(getRawChannelValue(AILERON));
			Serial.print("  CH2: "); Serial.print(getRawChannelValue(ELEVATOR));
			Serial.print("  CH3: "); Serial.print(getRawChannelValue(THROTTLE));
			Serial.print("  CH4: "); Serial.print(getRawChannelValue(RUDDER));
			Serial.print("  CH5: "); Serial.print(getRawChannelValue(AUX0));
			Serial.print("  CH6: "); Serial.println(getRawChannelValue(AUX1));
			Serial.print("Limit Error: "); Serial.print(pulseLengthOutsideLimitsN); 
			Serial.print("  Value: "); Serial.println(pulseLengthOutsideLimitsValue);
		}
	}
	#endif
}

void ch0Handler(){ pwmHandler(0, RECV_CHAN0PIN);};
void ch1Handler(){ pwmHandler(1, RECV_CHAN1PIN);};
void ch2Handler(){ pwmHandler(2, RECV_CHAN2PIN);};
void ch3Handler(){ pwmHandler(3, RECV_CHAN3PIN);};
void ch4Handler(){ pwmHandler(4, RECV_CHAN4PIN);};
void ch5Handler(){ pwmHandler(5, RECV_CHAN5PIN);};

void pwmHandler(int chann, int pin)
{ 
	unsigned int timeCurrentChange = TC2->TC_CHANNEL[2].TC_CV;

	// true if pins just went high
	if(digitalRead(pin))
	{
		timeLastChange[chann] = timeCurrentChange;
	}
	else
	{
		// clock runs at 42MHz therefore divide with 42 to get us
		unsigned int pulseLengthTmp = (timeCurrentChange - timeLastChange[chann])/42;

		// check if pulse lies within limits
		if(pulseLengthTmp >= RECV_PULSE_LENGTH_MIN && pulseLengthTmp <= RECV_PULSE_LENGTH_MAX)
		{
			pulseLength[chann] = pulseLengthTmp;
		}
		else
		{
			pulseLengthOutsideLimitsN++;
			pulseLengthOutsideLimitsValue = pulseLengthTmp;
		}
	}
}

unsigned int getRawChannelValue(enum recv_channel index)
{
	unsigned int output = pulseLength[index];

	/* I'm not sure if reading an unsigned int is atomic on the DUE. 
	 * It should be as the microprocessor is 32 bit.
	 * For the moment I'll assume it's a one clock operation.
	 * Otherwise: 
	 */
	//noInterrupts();
	//unsigned int output = pulseLength[index];
	//interrupts();	

	return output;
};