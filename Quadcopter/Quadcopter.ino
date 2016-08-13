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

#include <scheduler.h>
#include <flightController.h>

scheduler _scheduler;
flightController _flightController;

void setup()
{
  _scheduler.schedulerSetup();
  
  _flightController.initTimeCritical();
  _flightController.playStartUpSound();
  _flightController.startReceiverProcess();
  //_flightController.checkESCCalibration();
  _flightController.initSensors();
  _flightController.checkSensorTiltCalibration();
  _flightController.checkReceiverDefaultPosition();
  _flightController.estimateInitialAttitude();
  _flightController.playStartUpFinishedSound();

  _scheduler.setTime();
}

void loop()
{
  // 200Hz
  while(!_scheduler.checkBasePeriod());
  
  _flightController.readSensors();
  _flightController.callSensorFusion();
  _flightController.callStabilizer();
  _flightController.callMotors();

  // 100Hz
  if(_scheduler.checkDoublePeriod())
  {
  }

  // 66.66Hz
  if(_scheduler.checkTriplePeriod())
  {
  }

  // 50Hz
  if(_scheduler.checkQuadruplePeriod())
  {
    _flightController.readReceiver();
    _flightController.interpretReceiver();
  }

  // 20Hz
  if(_scheduler.checkDecuplePeriod())
  {
    _flightController.updateBuzzer();
    _flightController.readBatteryVoltage();
    _flightController.determineLoopControlSound();
  }  

  _scheduler.netExecTimeEnd();  
}
