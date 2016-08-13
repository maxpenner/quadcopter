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

#ifndef QC_BUZZER_H
#define QC_BUZZER_H

#define BUZZER_OUTPUT_PIN				5
#define BUZZER_OUTPUT_DUTYCYCLE			50		// percent
#define BUZZER_STD_PAUSE				50		// ms
#define BUZZER_PULSES_MAX				10
#define BUZZER_PULSES_LENGTH_MIN		100		// ms
#define BUZZER_PULSES_LENGTH_MAX		1000	// ms
#define BUZZER_FINAL_PAUSE_LENGTH_MAX	5000	// ms

class buzzer
{
	public:

		buzzer();
		~buzzer();
		
		// init arduino related stuff in setup
		void buzzerSetup();
		
		// continuous sound has a higher priority than a melody
		void startContinuousSound();
		void stopContinuousSound();
		
		// 'update()' has to be called periodically for non-blocking melodies
		bool startMelody(unsigned long _pulses, unsigned long _pulsesLength, unsigned long _finalPause, bool _blockingCall);
		void stopMelody();
		void update();
		
	private:
	
		bool continuousSoundSubscribed;
		bool melodySubscribed;
		
		// melody properties
		unsigned long pulses;
		unsigned long pulsesLength;
		unsigned long finalPause;
		bool blockingCall;
		
		// melody state machine variables
		int melodySubState;
		unsigned long melodySubStateStart;
		bool startFinalPause;
		
		void buzz();
		void buzzStop();
		
		bool pulse_x_fsm(unsigned long pulses, unsigned long pulseLength, unsigned long finalPauseLength);
};

#endif