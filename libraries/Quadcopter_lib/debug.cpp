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
#include "debug.h"

enclosedDbger::enclosedDbger(const char* startMessage)
{
	startTime = micros();
	DBG_println(startMessage);
}

enclosedDbger::~enclosedDbger()
{
}

void enclosedDbger::close(const char* stopMessage)
{
	DBG_print("Done after "); 
	DBG_print(micros() - startTime); 
	
	if(stopMessage == 0)
	{
		DBG_println(" us.");
	}
	else
	{
		DBG_print(" us. "); DBG_println(stopMessage);
	}
}