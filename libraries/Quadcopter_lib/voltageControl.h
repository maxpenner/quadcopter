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

/****************************************
*
*	Voltage control for battery voltage:
*
* 		3S Battery +_____________________
*			    						|
*										|
*									   –––
*									  |   |
*									  |R0 |
*									  |   |
*									   –––
*										|
*                       	    ________|
*								|		|
*							   –––	   –––
*							  |   |	  |   |
*							  |Z-D|	  |R1 |
*							  |5v |	  |   |
*							   –––	   –––
*			    				|		|________________ + Arduino Input
*								|		|
*								|	   –––
*								|	  |   |
*								|	  |R2 |
*								|	  |   |
*								|	   –––
*								|		|
* 		3S Battery -____________|_______|________________ - Arduino Input
* 
* 
* 		Z Diode Protects Arduino Input: Here 5.1 Volts and max current of 100mA (can also be 3.3 V with R1 = 0 Ohm).
* 
* 		Let's say we want to measure voltages up to 20 V.
* 
* 			R0 limits the current:
* 
* 			R0 > (20V - 5.1V)/100mA = 149 Ohm
* 				
* 			Let's say R0 = 1k. Maximum current then is (20-5.1)/1k = 14.9mA.
* 
* 		At 20 V battery voltage the Z-Diode should start limiting.
* 
* 			R12 = R1+R2.
* 
* 			R12/(R0 + R12)*20 = 5.1 -> R12 < R0/(20/5.1 - 1) = 342 Ohm ~ 300 Ohm
* 
* 		If the Z-Diode is limiting, the max. voltage over R2 is 3.3 Volts. Let's say even 2.5 Volts to be on the save side.
* 
* 			R2/R12*5.1 = 2.5 -> R2 = 2.5/5.1*300 = 147 Ohm ~ 150 Ohm
* 
* 			R1 = R12 - R2 = 150 Ohm
* 
* 		Scale factor in Arduino is:
* 
* 			scale = U_R2 / R2 * (R0+R1+R2) = U_R2 * 8.6666
* 
* 			Measure the voltage U_R2 over R2, multiply with 8.6666 and you have the battery voltage.
* 
* 		Minimum Resolution: 3.3/1024*8.6666 = 0.02792 V = 27.9 mV
* 
* 		Maximum Resolution: 3.3/4095*8.6666 = 0.0069 V = 6.9 mV
* 
* 		Maximum Measurable Voltage: 5.1/(R1+R2)*(R0+R1+R2) = 22.1 Volts. Above that voltage Arduino always shows the same value since Z-Diode is limiting.
*
******************************************/

#ifndef QC_VOLTAGECONTROL_H
#define QC_VOLTAGECONTROL_H

#define VOLTAGECONTROL_INPUT_PIN 			A0
#define VOLTAGECONTROL_MOVAVG_LENGTH_MAX 	10
#define VOLTAGECONTROL_SCALE 				8.6666f

class voltageControl
{
	public:

		voltageControl();
		~voltageControl();
		
		void voltageControlSetup();

		float getVoltage();
		void setMovAvgFilterLength(int mafl);
		
	private:
	
		int validValues;
		int movingAverageFilterLength;
		
		// memory on stack -> do not touch!
		unsigned int voltageValuesStack[VOLTAGECONTROL_MOVAVG_LENGTH_MAX];
		
		ringBuffer<unsigned int> voltageValues;
};

#endif