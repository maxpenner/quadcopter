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

#ifndef QC_FLIGHTCONTROLLER_H
#define QC_FLIGHTCONTROLLER_H

#include "vectorTypes.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS561101BA.h"
#include "sensorFusion.h"
#include "receiver.h"
#include "stabilizer.h"
#include "motors.h"
#include "buzzer.h"
#include "voltageControl.h"
#include "armingFSM.h"

class flightController
{
	public:
	
		flightController();
		~flightController();
		
		void initTimeCritical();
		void playStartUpSound();
		void startReceiverProcess();
		void checkESCCalibration();
		void initSensors();
		void checkSensorTiltCalibration();
		void checkReceiverDefaultPosition();
		void estimateInitialAttitude();
		void playStartUpFinishedSound();
		
		void readSensors();
		void callSensorFusion();
		void callStabilizer();
		void callMotors();
		void readReceiver();
		void interpretReceiver();
		void updateBuzzer();
		void readBatteryVoltage();
		void determineLoopControlSound();
		
	private:
	
		// subparts that flightcontroller needs
		MPU6050 _mpu6050;
		HMC5883L _hmc5883l;
		MS561101BA _ms561101ba;
		sensorFusion _sensorFusion;
		stabilizer _stabilizer;
		motors _motors;
		armingFSM _armingFSM;
		buzzer _buzzer;
		voltageControl _voltageControl;
	
		// quadcopter state variables
		bool releaseStabilizerOnMotors;
		
		// raw sensor data
		Vector3f accel;
		Vector3f gyros;
		Vector3f magne;
		Vector3f barom;
		
		// sensor fusion data
		Vector3f RPY_is;
		Vector3f RPYDot_is;
		float height_is;
		float heightDot_is;
		float heightDotDot_is;
		
		// raw clipped receiver input
		float roll_rx;
		float pitch_rx;
		float yaw_rx;
		float thrust_rx;
		float aux0_rx;
		float aux1_rx;
		
		// stabilizer output
		Vector4f pwm_output;
};

#endif