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

#ifndef QC_PID_H
#define QC_PID_H

class PID
{
	public:

		PID();
		~PID();
		
		void setGains(float kp_arg, float ki_arg, float kd_arg);
		void setiTermLimit(float iTermLimit_arg);
		void zeroErrorIntegral();
		
		float compute(float target, float is);

		float getErrorIntegral();
		
	private:
	
		// time between two calls of the filter
		float dT;
	
		// gains for P, I and D
		float kp, ki, kd;
		
		// needed for I
		float errorIntegral;
		float iTermLimit;		// integral wind-up security
		
		// needed for D
		float lastError;
};

#endif