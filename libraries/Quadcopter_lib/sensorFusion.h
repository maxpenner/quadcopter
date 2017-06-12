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
* Editorsetting from Notepad ++
*
* Earth coordinate system:
*
*				       +Z
*					   |
*					   | +PITCH
* 					   |  ^
*                      |  |
*                      |  |			  +ROLL
*                      |  |/			^
*                      |  /				|
*                      | /				|
*                      |/				|
*                      /----------------- +Y
*                     /(0,0,0)
*                    /
*                   /
*                  /-----> +YAW
*                 /
*                +X
*
*
* Body frame coordinate system:
*
*				       +Z
*					   |  +PITCH
*					   |    ^
* 					   |    |
*                      |    |/
*                      |  	/		   	  +ROLL
*                |M2|  |   /	  |M3|		^
*                      |  /					|
*                      | /					|
*                      |/					|
*                      /----------------------- +Y
*                     /(0,0,0)
*          |M1|      /         |M0|
*                   /
*                  /-----> +YAW
*                 /
*                +X
*
*			FRONT
* 
* This coordinate system is also used by the MPU6050 and the HMC5883L.
*
* Euler Angle system:
*
*		From Earth to Body Frame: YPR
*		From Body to Earth Frame: RPY
* 
* Angles in the Earth frame are called 'Euler Angles', in the body frame just angles.
*
*
******************************************/

#ifndef QC_SENSORFUSION_H
#define QC_SENSORFUSION_H

#include "debug.h"
#include "mathHelp.h"
#include "vectorTypes.h"
#include "complementaryFilter.h"

#define SFUS_SENSOR_TILT_DROPS	400
#define SFUS_SENSOR_TILT_READS	400

#define SFUS_INIT_ACCEL_DROPS	10
#define SFUS_INIT_ACCEL_READS	200
#define SFUS_INIT_GYROS_DROPS	10
#define SFUS_INIT_GYROS_READS	200
#define SFUS_INIT_MAGNE_DROPS	10
#define SFUS_INIT_MAGNE_READS	200
#define SFUS_INIT_BAROM_DROPS	10
#define SFUS_INIT_BAROM_READS	200

#define SFUS_ACCEL_BUF			10
#define SFUS_GYROS_BUF			10
#define SFUS_MAGNE_BUF			10
#define SFUS_BAROM_BUF			200

// Roll and pitch are adjusted proportionately to the total tilt angle.
// Sampling period is a privat class member.
#define SFUS_COMPF_RP_TAU_MIN	2.5f
#define SFUS_COMPF_RP_TAU_MAX	10.0f
#define SFUS_COMPF_RP_LIM_TILT	30.0f
#define SFUS_COMPF_YAW_A		1.0f
#define SFUS_COMPF_HEIGHTDOT_A	0.0f//0.9995f
#define SFUS_MA_HEIGHT			25

#define SFUS_SENSOR_TILT_R		-0.004363323f
#define SFUS_SENSOR_TILT_P		0.0f
#define SFUS_SENSOR_TILT_Y		0.0f

#define SFUS_USE_GYROS_CALIB

class sensorFusion
{
	public:

		sensorFusion();
		~sensorFusion();
		
		// can be called to calibrate tilt of chip on quadcopter
		bool writeSensorTiltCalibData(Vector3f accel, Vector3f magne);
		
		// must be called until true is returned
		bool writeInitData(Vector3f accel, Vector3f gyros, Vector3f magne, Vector3f barom);
		
		// input of raw sensor data
		void writeData(Vector3f accel, Vector3f gyros, Vector3f magne, Vector3f barom);
		
		Vector3f getRPY();
		Vector3f getRPYDot();
		float getHeight();
		float getHeightDot();
		float getHeightDotDot();
		float getStartHeight();
		
	private:
	
		Vector3f initSensorTiltAccelSum, initSensorTiltMagneSum;
		int initSensorTiltReadsCnt;
	
		Vector3f initAccelSum, initGyrosSum, initMagneSum, initBaromSum;
		int initAccelReadsCnt, initGyrosReadsCnt, initMagneReadsCnt, initBaromReadsCnt;
	
		// memory on stack -> do not touch!
		Vector3f accelDataStack[SFUS_ACCEL_BUF];
		Vector3f gyrosDataStack[SFUS_GYROS_BUF];
		Vector3f magneDataStack[SFUS_MAGNE_BUF];
		Vector3f baromDataStack[SFUS_BAROM_BUF];	
	
		// ring buffers will be connected to the stack memory
		ringBuffer<Vector3f> accelData, gyrosData, magneData, baromData;
		
		// time between two calls
		float dT;
		
		// filters for roll, pitch, yaw and heightdot
		float rollPitchA_minimum;
		float rollPitchA_maximum;
		complementaryFilter cf[4];
		
		// accel tilt compensation is rad
		Vector3f sensorTiltCalib;
		
		// gyro calibration in degrees/s
		Vector3f gyrosCalib;
		
		// fused attitude and altitude
		Vector3f RPY;
		Vector3f RPYDot;
		float height;
		float heightDot;
		float heightDotDot;
		float startHeight;
		
		// transformation functions
		float getTiltProportionalA();
		Vector3f getEulerAnglesFromAccel(Vector3f accelBodyFrame);
		float getEulerYawFromMagne(Vector3f magneBodyFrame);
		Vector3f getVectorFromBody2EarthFrame(Vector3f vecInBodyFrame, Vector3f attitude);
		Vector3f getRatesFromBody2EarthFrame(Vector3f ratesInBodyFrame, Vector3f attitude);
		
		#ifdef DBG_SFUS_RAW_PRINT
		unsigned long lastRawPrint;
		void printRawData(Vector3f a, Vector3f g, Vector3f m, Vector3f b);
		#endif
		
		#ifdef DBG_SFUS_FUSED_PRINT
		unsigned long lastSensorFusionPrint;
		void printSensorFusionData();
		#endif

		#ifdef DBG_SFUS_RANDOM_PRINT
		unsigned long lastSensorFusionRandomPrint;
		void printSensorFusionRandomData(Vector3f random0, Vector3f random1, Vector3f random2);
		#endif
};

#endif