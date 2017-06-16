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
#include <Wire.h>
#include "debug.h"
#include "scheduler.h"
#include "flightController.h"

flightController::flightController()
{
}

flightController::~flightController()
{	
}

void flightController::initTimeCritical()
{
	// I2C
	Wire.begin();
	Wire.setClock(400000);

	// debugging
	#ifdef DBG_ALLOW_SERIAL_DEBUGGING
	Serial.begin(250000);
	Serial.println("Arduino Due Quadcopter Firmware v1.0: Started in Debug Mode!");
	#endif	

	// PWM resolution
	analogWriteResolution(12);
	
	// Arduino related stuff that the subparts need
	_motors.motorsSetup();
	_buzzer.buzzerSetup();
	_voltageControl.voltageControlSetup();
	
	// ESCs start up first
	_motors.sendMinSignal();
	
	// during this delay ESCs should confirm minimum pulse length
	delay(2000);
	
	releaseStabilizerOnMotors = false;
}

void flightController::playStartUpSound()
{
	_buzzer.startMelody(2, 100, 0, BUZZER_BLOCKING);
	_buzzer.startMelody(1, 500, 0, BUZZER_BLOCKING);
}

void flightController::startReceiverProcess()
{
	enclosedDbger tmpDbg("\nFC: Starting receiver process...");
	
	receiverModule::init();
	delay(250);
	receiverModule::resetLimitStats();
	
	// check if transmitter and receiver are working properly
	while(!receiverModule::allChannelsInLimits())
	{
		DBG_println("\tReceiver pulse lengths are not within limits, check transmitter and receiver!");
		_buzzer.startMelody(1, 100, 1000, BUZZER_BLOCKING);
	}
	
	tmpDbg.close("250 ms startup.");
}

void flightController::checkESCCalibration()
{
	enclosedDbger tmpDbg("\nFC: Checking for ESC calibration...");
	
	readReceiver();
	
	// Check if all sticks are in default position AND throttle + AUX1 in max position.
	// AUX0 is ignored, used to signalize accel calibration.
	bool tmp = true;
	if(!receiverModule::checkLimitClippedValue(roll_rx, 1500.0f, 25.0f)) tmp = false;
	if(!receiverModule::checkLimitClippedValue(pitch_rx, 1500.0f, 25.0f)) tmp = false;
	if(!receiverModule::checkLimitClippedValue(thrust_rx, 2000.0f, 25.0f)) tmp = false;
	if(!receiverModule::checkLimitClippedValue(yaw_rx, 1500.0f, 25.0f)) tmp = false;	
	//if(!receiverModule::checkLimitClippedValue(aux0_rx, 1000.0f, 25.0f)) tmp = false;	
	if(!receiverModule::checkLimitClippedValue(aux1_rx, 2000.0f, 25.0f)) tmp = false;
		
	// if yes, start calibration process
	if(tmp == true)
	{
		_buzzer.startMelody(1, 1000, 0, BUZZER_BLOCKING);
		DBG_println("\tThrottle passed through to motors.");
		_motors.release();
		do
		{
			readReceiver();
			// thrust can be passed to motors because receiver and motors use same 1000 to 2000 interface
			_motors.feedStabilizerPWM(Vector4f(thrust_rx, thrust_rx, thrust_rx, thrust_rx));
			delay(50);
		}while(!receiverModule::checkLimitClippedValue(aux1_rx, 1000.0f, 25.0f));
		_motors.block();
		DBG_println("\tESC Calibration finished.");
	}
	else
	{
		DBG_println("\tNo ESC calibration demanded.");
	}
	
	DBG_println("\tSetting minimum ESC signal.");
	_motors.sendMinSignal();
	
	tmpDbg.close(0);
}

void flightController::initSensors()
{
	enclosedDbger tmpDbg("\nFC: Starting sensors...");
	
	// give sensors time to start up
	delay(500);
	
	while(!_mpu6050.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_4G))
	{
		DBG_println("\tCould not find a valid MPU6050 sensor, check wiring!");
		delay(250);
	}

	while(!_hmc5883l.begin())
	{
		DBG_println("\tCould not find a valid HMC5883L sensor, check wiring!");
		delay(250);
	}

	while(!_ms561101ba.begin(MS561101BA_ADDR_CSB_LOW, MS561101BA_OSR_4096))
	{
		DBG_println("\tCould not find a valid MS5611-01BA03 sensor, check wiring!");
		delay(250);
	}
	
	// give sensors time to complete the internal settings
	delay(500);
	
	tmpDbg.close("500ms startup + 50ms barom + 500ms final startup.");
}

void flightController::checkSensorTiltCalibration()
{
	enclosedDbger tmpDbg("\nFC: Checking if sensor tilt calibration desired...");
	
	readReceiver();
	
	// check if all sticks are in default position AND AUX0 is high
	bool tmp = true;
	if(!receiverModule::checkLimitClippedValue(roll_rx, 1500.0f, 25.0f)) tmp = false;
	if(!receiverModule::checkLimitClippedValue(pitch_rx, 1500.0f, 25.0f)) tmp = false;
	if(!receiverModule::checkLimitClippedValue(thrust_rx, 1000.0f, 25.0f)) tmp = false;
	if(!receiverModule::checkLimitClippedValue(yaw_rx, 1500.0f, 25.0f)) tmp = false;	
	if(!receiverModule::checkLimitClippedValue(aux0_rx, 2000.0f, 25.0f)) tmp = false;	
	if(!receiverModule::checkLimitClippedValue(aux1_rx, 1000.0f, 25.0f)) tmp = false;
	
	// if yes, start calibration process
	if(tmp == true)
	{
		_buzzer.startMelody(2, 1000, 0, BUZZER_BLOCKING);
		DBG_println("\tCalibrating sensor tilt.");
		scheduler tmp;
		tmp.schedulerSetup();
		do
		{
			while(!tmp.checkBasePeriod());
			readSensors();
		}
		while(!_sensorFusion.writeSensorTiltCalibData(accel, magne));
		DBG_println("\tValues are NOT permanent, only for this flight.");
		DBG_println("\tFinish process by setting AUX0 to default position.");
		do
		{
			readReceiver();
			delay(100);
		}while(!receiverModule::checkLimitClippedValue(aux0_rx, 1000.0f, 25.0f));
	}
	else
	{
		DBG_println("\tNo sensor tilt calibration demanded.");
	}
	
	tmpDbg.close(0);
}

void flightController::checkReceiverDefaultPosition()
{
	enclosedDbger tmpDbg("\nFC: Checking receiver default position...");
	
	// check if sticks are in default position
	while(1)
	{
		bool tmp = true;
		
		readReceiver();
		
		if(!receiverModule::checkLimitClippedValue(roll_rx, 1500.0f, 25.0f))
		{
			DBG_println("\tSet roll to default position!");
			tmp = false;
		}
		
		if(!receiverModule::checkLimitClippedValue(pitch_rx, 1500.0f, 25.0f))
		{
			DBG_println("\tSet pitch to default position!");
			tmp = false;
		}
		
		if(!receiverModule::checkLimitClippedValue(thrust_rx, 1000.0f, 25.0f))
		{
			DBG_println("\tSet thrust to default position!");
			tmp = false;
		}
		
		if(!receiverModule::checkLimitClippedValue(yaw_rx, 1500.0f, 25.0f))
		{
			DBG_println("\tSet rudder to default position!");
			tmp = false;
		}
				
		if(!receiverModule::checkLimitClippedValue(aux0_rx, 1000.0f, 25.0f))
		{
			DBG_println("\tSet aux0 to default position!");
			tmp = false;
		}
				
		if(!receiverModule::checkLimitClippedValue(aux1_rx, 1000.0f, 25.0f))
		{
			DBG_println("\tSet aux1 to default position!");
			tmp = false;
		}
		
		if(tmp == true)
			break;
		else
			_buzzer.startMelody(2, 100, 1000, BUZZER_BLOCKING);
	}
	
	tmpDbg.close(0);
}

void flightController::estimateInitialAttitude()
{
	enclosedDbger tmpDbg("\nFC: Estimating initial attitude...");
	
	scheduler tmp;
	tmp.schedulerSetup();

	do
	{
		while(!tmp.checkBasePeriod());
		readSensors();
	}
	while(!_sensorFusion.writeInitData(accel, gyros, magne, barom));
	
	RPY_is = _sensorFusion.getRPY();
	RPYDot_is = _sensorFusion.getRPYDot();
	height_is = _sensorFusion.getHeight();
	heightDot_is = _sensorFusion.getHeightDot();
	heightDotDot_is = _sensorFusion.getHeightDotDot();
	
	_stabilizer.setInitYawLock(RPY_is.z);
	
	tmpDbg.close(0);
}

void flightController::playStartUpFinishedSound()
{
	_buzzer.startMelody(2, 100, 0, BUZZER_BLOCKING);
	_buzzer.startMelody(2, 500, 0, BUZZER_BLOCKING);
}

void flightController::readSensors()
{
	accel = _mpu6050.readNpkgAccel();
	gyros = _mpu6050.readRpsGyro();
	magne = Vector3f(20.0f, 0.0f, -40.0f); //_hmc5883l.readCalibrated();
	barom = _ms561101ba.getTPA(true);
}

void flightController::callSensorFusion()
{
	_sensorFusion.writeData(accel, gyros, magne, barom);

	RPY_is = _sensorFusion.getRPY();
	RPYDot_is = _sensorFusion.getRPYDot();
	height_is = _sensorFusion.getHeight();
	heightDot_is = _sensorFusion.getHeightDot();
	heightDotDot_is = _sensorFusion.getHeightDotDot();
}

void flightController::callStabilizer()
{
	// Stabilizer continuously calculates motors speeds that are needed to lean or level the quadcopter.
	// The output is limited to the minimum spinning speed and full speed.
	pwm_output = _stabilizer.compute_pwmDutyCycle(RPY_is, RPYDot_is, height_is, heightDot_is, heightDotDot_is, roll_rx, pitch_rx, yaw_rx, thrust_rx);	
}

void flightController::callMotors()
{
	// artificially set motor speeds to zero (stillstand)
	if(!releaseStabilizerOnMotors)
		pwm_output = Vector4f(MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MIN, MOTORS_ESC_PWM_MIN);	
	
	_motors.feedStabilizerPWM(pwm_output);
	
	// usefull line for esc calibration
	//_motors.feedStabilizerPWM(Vector4f(thrust_rx, thrust_rx, thrust_rx, thrust_rx));
}

void flightController::readReceiver()
{
	roll_rx = receiverModule::getClippedChannelValue_f(AILERON);
	pitch_rx = receiverModule::getClippedChannelValue_f(ELEVATOR);
	yaw_rx = receiverModule::getClippedChannelValue_f(RUDDER);
	thrust_rx = receiverModule::getClippedChannelValue_f(THROTTLE);
	aux0_rx = receiverModule::getClippedChannelValue_f(AUX0);
	aux1_rx = receiverModule::getClippedChannelValue_f(AUX1);
	
	#ifdef DBG_RECEIVER_PRINT
	receiverModule::printDebug();
	#endif
}

void flightController::interpretReceiver()
{
	// always check panic stop first
	if(receiverModule::checkZoneClippedValue(aux0_rx, 2) == 1)
	{
		_motors.panicStop();
	}
	
	// armed?
	if(_armingFSM.refreshArmingState(thrust_rx, yaw_rx))
	{
		_motors.release();
		
		if((thrust_rx >= RECV_CLIP_MINIMUM + 100.0f))
		{
			// Make thrust less sensitive.
			// No mapping from receiver to motors needed since they both have an 1000 to 2000 interface.
			thrust_rx -= RECV_CLIP_MINIMUM + 100.0f;
			thrust_rx *= 0.8f;
			thrust_rx += RECV_CLIP_MINIMUM + 100.0f;
			thrust_rx = constrainn<float>(thrust_rx, RECV_CLIP_MINIMUM, RECV_CLIP_MAXIMUM);
			
			releaseStabilizerOnMotors = true;
		}
		else
		{
			_stabilizer.setInitYawLock(RPY_is.z);
			_stabilizer.resetIntegrals();
		}
		
		// landing hysteresis
		if(thrust_rx <= RECV_CLIP_MINIMUM + 50)
		{
			releaseStabilizerOnMotors = false;
		}
		
		// setting the flight mode via AUX1
		if(receiverModule::checkZoneClippedValue(aux1_rx, 3) == 1)
		{
			_stabilizer.setFlightMode(STABILIZE);
		}
		else if((receiverModule::checkZoneClippedValue(aux1_rx, 3) == 2))// && (height_is > _sensorFusion.getStartHeight() + 2.0f))
		{
			_stabilizer.setFlightMode(STABILIZE_HEIGHT);
		}
	}
	else
	{
		releaseStabilizerOnMotors = false;
		_motors.block();
	}
}

void flightController::updateBuzzer()
{
	_buzzer.update();
}

void flightController::readBatteryVoltage()
{
	float batteryVoltage = _voltageControl.getVoltage();
}

void flightController::determineLoopControlSound()
{
	if((_armingFSM.getArmingProcessState() == ARMING) || (_armingFSM.getArmingProcessState() == DISARMING))
	{
		_buzzer.startContinuousSound();
	}
	else
	{
		_buzzer.stopContinuousSound();
	}
	
	// quadcopter is on the ground and armed, but rotors aren't spinning
	if((_armingFSM.getArmingState() == true) && (releaseStabilizerOnMotors == false))
	{
		_buzzer.startMelody(1, 200, 1000, BUZZER_NON_BLOCKING);
	}
}