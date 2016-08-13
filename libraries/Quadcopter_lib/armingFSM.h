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

#ifndef QC_ARMINGFSM_H
#define QC_ARMINGFSM_H

#define ARMINGFMS_ARMING_SIGNAL_LENGTH		1200	// ms
#define ARMINGFMS_DISARMING_SIGNAL_LENGTH	1200	// ms

typedef enum
{
	WAITING = 0,
	ARMING = 1,
	DISARMING = 2,
	BLOCKING = 3
}armingFSM_process;

class armingFSM
{
	public:

		armingFSM();
		~armingFSM();

		// This state machine has to be feed continuously.
		// Arguments come from the receiver.
		// True if armed, false if disarmed.
		bool refreshArmingState(float thrust_rx, float yaw_rx);
		
		bool getArmingState();
		armingFSM_process getArmingProcessState();
		
	private:
	
		bool armed;
		unsigned long armingStart;
		armingFSM_process armingProcessState;
		
		bool checkLimitValue(float value, float center, float limits);
};

#endif