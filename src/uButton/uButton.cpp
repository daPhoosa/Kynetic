/*
	Button library that includes de-bounce
	Phillip Schmidt, 02/14
	v1.0
	
         This program is free software: you can redistribute it and/or modify
         it under the terms of the GNU General Public License as published by
         the Free Software Foundation, either version 3 of the License, or
         (at your option) any later version.

         This program is distributed in the hope that it will be useful,
         but WITHOUT ANY WARRANTY; without even the implied warranty of
         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
         GNU General Public License for more details.

         You should have received a copy of the GNU General Public License
         along with this program.  If not, see <http://www.gnu.org/licenses/>

*/

#include "uButton.h"


uButton::uButton(int pinNum, int normState){	// input pin, input normal state

	pin = pinNum;

	if(normState == 0){				// normal state of input is LOW
		pinMode(pin, INPUT);			// switch to v+: switch closed = input HIGH (requires external pull-down resistor)
		switchDown = HIGH;
	}else{								// normal state of input is HIGH
		pinMode(pin, INPUT_PULLUP);// switch to ground: switch closed = input LOW (set internal pull-up resistor)
		switchDown = LOW;
	}

	stateLast = digitalRead(pin);
}


bool uButton::check()
{
	bool stateNow = digitalRead(pin);

	// the same reading must be seen twice before action is taken

	if( stateNow != switchDown && stateLast != switchDown ) // btn released check
	{
		notRun = true;
		return false;
	}
	
	if( stateNow == switchDown && stateLast == switchDown && notRun ) // btn pressed check
	{
		notRun = false;
		return true;	// only return true once per press
	}

	stateLast = stateNow; // this only executes when a state change is observed
	return false;
}

