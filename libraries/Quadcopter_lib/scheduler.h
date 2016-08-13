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

#ifndef QC_SCHEDULER_H
#define QC_SCHEDULER_H

class scheduler
{
	public:

		scheduler();
		~scheduler();
		
		void schedulerSetup();

		void setTime();							// resets the scheduler time
		bool checkBasePeriod();
		bool checkDoublePeriod();
		bool checkTriplePeriod();
		bool checkQuadruplePeriod();
		bool checkDecuplePeriod();
		
		void netExecTimeEnd();					// call when net execution time of one period is over
		unsigned long getPeriodTimeMin();
		unsigned long getPeriodTimeMax();
		unsigned long getNetExecTimeMin();
		unsigned long getNetExecTimeMax();
		
	private:
	
		unsigned long programStartTime;			// us
		unsigned long lastPeriodBegin;
		int doublePeriod;
		int triplePeriod;
		int quadruplePeriod;
		int decuplePeriod;
		
		// statistics for scheduler control
		unsigned long periodTimeMin;			// us
		unsigned long periodTimeMax;
		unsigned long firstExecTime;
		unsigned long netExecTimeMin;
		unsigned long netExecTimeMax;
		
		#ifdef DBG_SCHEDULER_PIN
		bool pinState;
		#endif
		
		#ifdef DBG_SCHEDULER_PRINT
		unsigned long lastExecTimePrint;
		#endif
};

#endif