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
#include "global.h"
#include "sensorFusion.h"

#if defined(DBG_SFUS_TILT_CALIB_PRINT) || defined(DBG_SFUS_INIT_ATTI_PRINT) || defined(DBG_SFUS_RAW_PRINT) || defined(DBG_SFUS_FUSED_PRINT) || defined(DBG_SFUS_RANDOM_PRINT)
#include "Arduino.h"
#endif

#define ROLL 0
#define PITCH 1
#define YAW 2
#define HEIGHTDOT 3

sensorFusion::sensorFusion()
{
	sensorTiltCalib = Vector3f(SFUS_SENSOR_TILT_R, SFUS_SENSOR_TILT_P, SFUS_SENSOR_TILT_Y);
	initSensorTiltAccelSum.reset();
	initSensorTiltMagneSum.reset();
	initSensorTiltReadsCnt = 0;
	
	initAccelSum.reset();
	initGyrosSum.reset();
	initMagneSum.reset();
	initBaromSum.reset();
	initAccelReadsCnt = 0;
	initGyrosReadsCnt = 0;
	initMagneReadsCnt = 0;
	initBaromReadsCnt = 0;
	
	// couple ring buffers with stack memory
	accelData = ringBuffer<Vector3f>(accelDataStack, SFUS_ACCEL_BUF);
	gyrosData = ringBuffer<Vector3f>(gyrosDataStack, SFUS_GYROS_BUF);
	magneData = ringBuffer<Vector3f>(magneDataStack, SFUS_MAGNE_BUF);
	baromData = ringBuffer<Vector3f>(baromDataStack, SFUS_BAROM_BUF);
	
	dT = GLB_LOOP_TIME/1000000.0f;
	
	rollPitchA_minimum = SFUS_COMPF_RP_TAU_MIN/(SFUS_COMPF_RP_TAU_MIN + dT);
	rollPitchA_maximum = SFUS_COMPF_RP_TAU_MAX/(SFUS_COMPF_RP_TAU_MAX + dT);

	// init complementary filter
	cf[ROLL].setTauViaA(rollPitchA_minimum);
	cf[PITCH].setTauViaA(rollPitchA_minimum);
	cf[YAW].setTauViaA(SFUS_COMPF_YAW_A);
	cf[HEIGHTDOT].setTauViaA(SFUS_COMPF_HEIGHTDOT_A);
	
	#ifdef DBG_SFUS_RAW_PRINT
	lastRawPrint = 0;
	#endif
	
	#ifdef DBG_SFUS_FUSED_PRINT
	lastSensorFusionPrint = 0;
	#endif
	
	#ifdef DBG_SFUS_RANDOM_PRINT
	lastSensorFusionRandomPrint = 0;
	#endif	
}

sensorFusion::~sensorFusion()
{
}

bool sensorFusion::writeSensorTiltCalibData(Vector3f accel, Vector3f magne)
{
	initSensorTiltReadsCnt++;
	if(initSensorTiltReadsCnt > SFUS_SENSOR_TILT_DROPS && initSensorTiltReadsCnt <= (SFUS_SENSOR_TILT_DROPS + SFUS_SENSOR_TILT_READS))
	{
		initSensorTiltAccelSum += accel;
		initSensorTiltMagneSum += magne;
	}

	// data collection finished
	if(initSensorTiltReadsCnt >= (SFUS_SENSOR_TILT_DROPS + SFUS_SENSOR_TILT_READS))
	{
		initSensorTiltAccelSum /= SFUS_SENSOR_TILT_READS;
		initSensorTiltMagneSum /= SFUS_SENSOR_TILT_READS;
		
		sensorTiltCalib.reset();

		// quadcopter stands flat on the ground and is directed to magnetic north
		RPY.reset();
		
		// calculate roll and pitch compensation from accelerometer
		sensorTiltCalib = getEulerAnglesFromAccel(initSensorTiltAccelSum);
		
		// compensate magnetometer roll and pitch tilt on quadcopter
		Vector3f initSensorTiltMagneSumRPCompensated = getVectorFromBody2EarthFrame(initSensorTiltMagneSum, sensorTiltCalib);
		
		// calculate yaw tilt of magnetometer relative to quadcopter
		sensorTiltCalib.z = getEulerYawFromMagne(initSensorTiltMagneSumRPCompensated);
		
		#ifdef DBG_SFUS_TILT_CALIB_PRINT
		Serial.print("\tTilt (Degrees): r = "); Serial.print(sensorTiltCalib.x*RAD2DEG); Serial.print(" p = "); Serial.print(sensorTiltCalib.y*RAD2DEG);
		Serial.print(" y = "); Serial.println(sensorTiltCalib.z*RAD2DEG);
		#endif
		
		return true;		
	}	
	
	return false;
}

bool sensorFusion::writeInitData(Vector3f accel, Vector3f gyros, Vector3f magne, Vector3f barom)
{
	initAccelReadsCnt++;
	if(initAccelReadsCnt > SFUS_INIT_ACCEL_DROPS && initAccelReadsCnt <= (SFUS_INIT_ACCEL_DROPS + SFUS_INIT_ACCEL_READS))
		initAccelSum += accel;
		
	initGyrosReadsCnt++;
	if(initGyrosReadsCnt > SFUS_INIT_GYROS_DROPS && initGyrosReadsCnt <= (SFUS_INIT_GYROS_DROPS + SFUS_INIT_GYROS_READS))
		initGyrosSum += gyros;
	
	initMagneReadsCnt++;
	if(initMagneReadsCnt > SFUS_INIT_MAGNE_DROPS && initMagneReadsCnt <= (SFUS_INIT_MAGNE_DROPS + SFUS_INIT_MAGNE_READS))
		initMagneSum += magne;
	
	if(barom.valid == true)
	{
		initBaromReadsCnt++;
		if(initBaromReadsCnt > SFUS_INIT_BAROM_DROPS && initBaromReadsCnt <= (SFUS_INIT_BAROM_DROPS + SFUS_INIT_BAROM_READS))
			initBaromSum += barom;	
	}

	// data collection finished
	if(initAccelReadsCnt >= (SFUS_INIT_ACCEL_DROPS + SFUS_INIT_ACCEL_READS)
		&& initGyrosReadsCnt >= (SFUS_INIT_GYROS_DROPS + SFUS_INIT_GYROS_READS)
			&& initMagneReadsCnt >= (SFUS_INIT_MAGNE_DROPS + SFUS_INIT_MAGNE_READS)
				&& initBaromReadsCnt >= (SFUS_INIT_BAROM_DROPS + SFUS_INIT_BAROM_READS))
	{
		initAccelSum /= SFUS_INIT_ACCEL_READS;
		initGyrosSum /= SFUS_INIT_GYROS_READS;
		initMagneSum /= SFUS_INIT_MAGNE_READS;
		initBaromSum /= SFUS_INIT_BAROM_READS;
		
		// compensate chip tilt
		initAccelSum = getVectorFromBody2EarthFrame(initAccelSum, sensorTiltCalib);
		initMagneSum = getVectorFromBody2EarthFrame(initMagneSum, sensorTiltCalib);
		
		// determine initial values
		RPY = getEulerAnglesFromAccel(initAccelSum);
		RPY.z = getEulerYawFromMagne(initMagneSum);
		RPYDot = Vector3f(0.0f, 0.0f, 0.0f);
		height = initBaromSum.z;
		heightDot = 0.0f;
		heightDotDot = 0.0f;
		startHeight = height;
		
		#ifdef SFUS_USE_GYROS_CALIB
		gyrosCalib = initGyrosSum;
		#else
		gyrosCalib = Vector3f(0.0f, 0.0f, 0.0f);
		#endif
		
		// push values into buffers
		for(int i=0; i<SFUS_ACCEL_BUF; i++) accelData.pushNewElem(initAccelSum);
		for(int i=0; i<SFUS_GYROS_BUF; i++) gyrosData.pushNewElem(initGyrosSum);
		for(int i=0; i<SFUS_MAGNE_BUF; i++) magneData.pushNewElem(initMagneSum);
		for(int i=0; i<SFUS_BAROM_BUF; i++) baromData.pushNewElem(initBaromSum);		
		
		// init complementary filter A/Tau factor
		float currentA = getTiltProportionalA();
		cf[ROLL].setTauViaA(currentA);
		cf[PITCH].setTauViaA(currentA);			
		
		// init complementary filter angles
		cf[ROLL].setCombinedEstimation(RPY.x);
		cf[PITCH].setCombinedEstimation(RPY.y);
		cf[YAW].setCombinedEstimation(RPY.z);
		cf[HEIGHTDOT].setCombinedEstimation(heightDot);
		
		#ifdef DBG_SFUS_INIT_ATTI_PRINT
		Serial.print("\nInit Attitude (Degrees): r = "); Serial.print(RPY.x*RAD2DEG); Serial.print(" p = "); Serial.print(RPY.y*RAD2DEG);
		Serial.print(" y = "); Serial.print(RPY.z*RAD2DEG); Serial.print(" h = "); Serial.println(height);
		Serial.print("Gyroscope calibration: r = "); Serial.print(gyrosCalib.x*RAD2DEG); Serial.print(" p = "); Serial.print(gyrosCalib.y*RAD2DEG);
		Serial.print(" y = "); Serial.println(gyrosCalib.z*RAD2DEG);
		#endif
		
		return true;		
	}	
	
	return false;
}

void sensorFusion::writeData(Vector3f accel, Vector3f gyros, Vector3f magne, Vector3f barom)
{
	#ifdef DBG_SFUS_RAW_PRINT
	printRawData(accel, gyros, magne, barom);
	#endif	
	
	// compensate chip tilt for all sensors
	accel = getVectorFromBody2EarthFrame(accel, sensorTiltCalib);
	gyros = getRatesFromBody2EarthFrame(gyros-gyrosCalib, sensorTiltCalib);
	magne = getVectorFromBody2EarthFrame(magne, sensorTiltCalib);
	
	// push tilt compensated values to container
	accelData.pushNewElem(accel);
	gyrosData.pushNewElem(gyros);
	magneData.pushNewElem(magne);
	if(barom.valid == true)
	{
		baromData.pushNewElem(barom);
	}
	else
	{
		// if value is not valid push old value -> this does not happen very often, so the effect should be negligible
		Vector3f tmp_invalid;
		baromData.getNthElem(tmp_invalid, 0);
		baromData.pushNewElem(tmp_invalid);
	}
	
	// transform input from sensors into earth frame
	Vector3f accelEulertmp = getEulerAnglesFromAccel(accel);
	Vector3f gyrosEulerRatestmp = getRatesFromBody2EarthFrame(gyros, RPY);
	Vector3f accelAccelerationtmp = getVectorFromBody2EarthFrame(accel, RPY);
	
	// adjust complementary filter
	float currentA = getTiltProportionalA();
	cf[ROLL].setTauViaA(currentA);
	cf[PITCH].setTauViaA(currentA);
	
	// filtering for roll and pitch
	RPY.x = cf[ROLL].getCombinedEstimation(accelEulertmp.x, gyrosEulerRatestmp.x);
	RPY.y = cf[PITCH].getCombinedEstimation(accelEulertmp.y, gyrosEulerRatestmp.y);
	RPY.z = cf[YAW].getCombinedEstimation(getEulerYawFromMagne(magne), gyrosEulerRatestmp.z);
	
	// yaw is +/- Pi
	RPY.z = wrap_Pi(RPY.z);
	
	// complementary filter does not know it has to wrap_Pi
	cf[YAW].setCombinedEstimation(RPY.z);
	
	// rotation rates from calibrated gyroscope
	RPYDot = gyrosEulerRatestmp;
	
	// apply moving average filter for height
	height = 0.0f;
	float height_old = 0.0f;
	for(int i=0; i<SFUS_MA_HEIGHT; i++)
	{
		Vector3f tmp;
		
		baromData.getNthElem(tmp, i);
		height += tmp.z;
		
		baromData.getNthElem(tmp, i+1);
		height_old += tmp.z;
	}
	height /= SFUS_MA_HEIGHT;
	height_old /= SFUS_MA_HEIGHT;
	
	// vertical speed
	heightDot = cf[HEIGHTDOT].getCombinedEstimation((height - height_old)/dT, accelAccelerationtmp.z - GRAVITY);

	// vertical acceleration
	heightDotDot = accelAccelerationtmp.z - GRAVITY;
	
	#ifdef DBG_SFUS_FUSED_PRINT
	printSensorFusionData();
	#endif
	
	#ifdef DBG_SFUS_RANDOM_PRINT
	printSensorFusionRandomData(magne, Vector3f(accelEulertmp.x*RAD2DEG, accelEulertmp.y*RAD2DEG, 0.0f), Vector3f(0.0f, 0.0f, 0.0f));
	#endif
}

Vector3f sensorFusion::getRPY()
{
	return RPY;
}

Vector3f sensorFusion::getRPYDot()
{
	return RPYDot;
}

float sensorFusion::getHeight()
{
	return height;
}

float sensorFusion::getHeightDot()
{
	return heightDot;
}

float sensorFusion::getHeightDotDot()
{
	return heightDotDot;
}

float sensorFusion::getStartHeight()
{
	return startHeight;
}

float sensorFusion::getTiltProportionalA()
{	
	// calculate the total tilt angle (pitch and roll)
	float T_total = RAD2DEG*(acos(cos(RPY.x)*cos(RPY.y)));
	
	// limit the tilt angle
	T_total = constrainn<float>(T_total, 0.0f, SFUS_COMPF_RP_LIM_TILT);
	
	return mapp<float>(T_total, 0.0f, SFUS_COMPF_RP_LIM_TILT, rollPitchA_minimum, rollPitchA_maximum);
}

Vector3f sensorFusion::getEulerAnglesFromAccel(Vector3f accelBodyFrame)
{
	// Source: https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
	float roll_accel = (accelBodyFrame.z != 0.0f) ? atan(accelBodyFrame.y/accelBodyFrame.z) : 0.0f;
	float tmp = sqrt(accelBodyFrame.y*accelBodyFrame.y + accelBodyFrame.z*accelBodyFrame.z);
	float pitch_accel = (tmp != 0.0f) ? atan(-accelBodyFrame.x/tmp) : 0.0f;	

	return Vector3f(roll_accel, pitch_accel, 0.0f);	
}

float sensorFusion::getEulerYawFromMagne(Vector3f magneBodyFrame)
{
	// tilt compensation without yaw
	// Source: http://www.chrobotics.com/library/understanding-euler-angles
	// Source: http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	Vector3f magneEarthFrame = getVectorFromBody2EarthFrame(magneBodyFrame, Vector3f(RPY.x, RPY.y, 0.0f));
	
	// Source: https://www.adafruit.com/datasheets/AN203_Compass_Heading_Using_Magnetometers.pdf
	float orientation_tmp;
	if(magneEarthFrame.y == 0.0f)
	{
		if(magneEarthFrame.x > 0.0f)
			orientation_tmp = 0.0f;
		else if(magneEarthFrame.x < 0.0f)
			orientation_tmp = Pi;
	}
	else if(magneEarthFrame.y > 0.0f)
	{
		orientation_tmp = Pi/2.0f - atan(magneEarthFrame.x/magneEarthFrame.y);
	}
	else if(magneEarthFrame.y < 0.0f)
	{
		orientation_tmp = 1.5f*Pi - atan(magneEarthFrame.x/magneEarthFrame.y);
	}

	// transform from 0-2*Pi to +/-Pi
	return wrap_Pi(-orientation_tmp);
}

Vector3f sensorFusion::getVectorFromBody2EarthFrame(Vector3f vecInBodyFrame, Vector3f attitude)
{
	// The angles are negative, but the minuses are itegrated in the matrix.
	// Source: http://www.chrobotics.com/library/understanding-euler-angles
	float cos_roll = cos(attitude.x);
	float sin_roll = sin(attitude.x);
	float cos_pitch = cos(attitude.y);
	float sin_pitch = sin(attitude.y);
	float cos_yaw = cos(attitude.z);
	float sin_yaw = sin(attitude.z);
	
	Vector3f vecInEarthFrame;
	
	vecInEarthFrame.x = cos_yaw*cos_pitch*vecInBodyFrame.x + (cos_yaw*sin_pitch*sin_roll-sin_yaw*cos_roll)*vecInBodyFrame.y + (cos_yaw*sin_pitch*cos_roll+sin_yaw*sin_roll)*vecInBodyFrame.z;
	vecInEarthFrame.y = sin_yaw*cos_pitch*vecInBodyFrame.x + (sin_yaw*sin_pitch*sin_roll+cos_yaw*cos_roll)*vecInBodyFrame.y + (sin_yaw*sin_pitch*cos_roll-cos_yaw*sin_roll)*vecInBodyFrame.z;
	vecInEarthFrame.z =        -sin_pitch*vecInBodyFrame.x +          cos_pitch*sin_roll                  *vecInBodyFrame.y +          cos_pitch*cos_roll                  *vecInBodyFrame.z;
	
	return vecInEarthFrame;	
}

Vector3f sensorFusion::getRatesFromBody2EarthFrame(Vector3f ratesInBodyFrame, Vector3f attitude)
{
	/* Transform the rates from the body frame into the earth frame.
	 * To do so, you need to know the current roll and pitch.
	 * The angles are negative, but the minuses are itegrated in the matrix.
	 * Can be ignored in a small angle approximation.
	 * Source: http://www.chrobotics.com/library/understanding-euler-angles
	 * Source: http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
	 */
	
	float sin_roll = sin(attitude.x);
	float cos_roll = cos(attitude.x);
	float tan_pitch = tan(attitude.y);
	float cos_pitch = cos(attitude.y);
	
	Vector3f ratesInEarthFrame;
	
	ratesInEarthFrame.x = ratesInBodyFrame.x + sin_roll*tan_pitch*ratesInBodyFrame.y + cos_roll*tan_pitch*ratesInBodyFrame.z;
	ratesInEarthFrame.y =                      cos_roll          *ratesInBodyFrame.y - sin_roll          *ratesInBodyFrame.z;
	ratesInEarthFrame.z =                      sin_roll/cos_pitch*ratesInBodyFrame.y + cos_roll/cos_pitch*ratesInBodyFrame.z;

	return ratesInEarthFrame;
}

#ifdef DBG_SFUS_RAW_PRINT
void sensorFusion::printRawData(Vector3f a, Vector3f g, Vector3f m, Vector3f b)
{
	unsigned long now = micros();
	
	if(now - lastRawPrint >= DBG_SFUS_RAW_PRINT)
	{
		lastRawPrint = now;
		
		Serial.print("\nRaw A: x = "); Serial.print(a.x); Serial.print(" y = "); Serial.print(a.y); Serial.print(" z = "); Serial.println(a.z);
		Serial.print("Raw G: x = "); Serial.print(g.x*RAD2DEG); Serial.print(" y = "); Serial.print(g.y*RAD2DEG); Serial.print(" z = "); Serial.println(g.z*RAD2DEG);
		Serial.print("Raw M: x = "); Serial.print(m.x); Serial.print(" y = "); Serial.print(m.y); Serial.print(" z = "); Serial.println(m.z);
		Serial.print("Raw B: t = "); Serial.print(b.x); Serial.print(" p = "); Serial.print(b.y); Serial.print(" a = "); Serial.println(b.z);
	}
}
#endif

#ifdef DBG_SFUS_FUSED_PRINT
void sensorFusion::printSensorFusionData()
{
	unsigned long now = micros();
	
	if(now - lastSensorFusionPrint >= DBG_SFUS_FUSED_PRINT)
	{
		lastSensorFusionPrint = now;
		
		Serial.print("\nSF R = "); Serial.print(RPY.x*RAD2DEG); Serial.print(" P = "); Serial.print(RPY.y*RAD2DEG); Serial.print(" Y = "); Serial.println(RPY.z*RAD2DEG);
		Serial.print("SF H = "); Serial.print(height); Serial.print(" HD = "); Serial.print(heightDot); Serial.print(" HDD = "); Serial.println(heightDotDot);
		Serial.print("SF A = "); Serial.println(getTiltProportionalA()*1000.0f);
	}
}
#endif

#ifdef DBG_SFUS_RANDOM_PRINT
void sensorFusion::printSensorFusionRandomData(Vector3f random0, Vector3f random1, Vector3f random2)
{
	unsigned long now = micros();
	
	if(now - lastSensorFusionRandomPrint >= DBG_SFUS_RANDOM_PRINT)
	{
		lastSensorFusionRandomPrint = now;
		
		Serial.print("\nR0 x = "); Serial.print(random0.x); Serial.print(" y = "); Serial.print(random0.y); Serial.print(" z = "); Serial.println(random0.z);
		Serial.print("R1 x = "); Serial.print(random1.x); Serial.print(" y = "); Serial.print(random1.y); Serial.print(" z = "); Serial.println(random1.z);
		Serial.print("R2 x = "); Serial.print(random2.x); Serial.print(" y = "); Serial.print(random2.y); Serial.print(" z = "); Serial.println(random2.z);
	}
}
#endif