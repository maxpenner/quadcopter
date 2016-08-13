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

// Algorithm for first order filter: https://b94be14129454da9cf7f056f5f8b89a9b17da0be.googledrive.com/host/0B0ZbiLZrqVa6Y2d3UjFVWDhNZms/filter.pdf

#ifndef QC_COMPLEMENTARYFILTER_H
#define QC_COMPLEMENTARYFILTER_H

#define COMPLEMENTARY_FILTER_TAU_UNDEFINED	-1.0f	// if tau converges to +infinity

class complementaryFilter
{
	public:

		complementaryFilter();
		~complementaryFilter();
		
		// set starting point
		void setCombinedEstimation(float combEstim);		
		
		// extract the most recent estimation
		float getCombinedEstimation(float estimation, float estimation_derivative);
		
		// Get and set parameter 'a' respectively 'tau'.
		// 0 <= tau < +infinity
		// 0 <= a <= 1
		float getTau();
		void setTau(float tau_arg);
		void setTauViaA(float a_arg);
		float getA();
		
	private:
	
		// combined estimatation of both inputs
		float combinedEstimation;
		
		// time between two samples
		float dT;
		
		// relationship: a = tau/(tau+dT) respectively tau = a*dT/(1-a)
		float tau;
		float a;
};

#endif