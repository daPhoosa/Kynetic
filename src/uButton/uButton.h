/*
	Button library
	Phillip Schmidt, 02/17
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

			
	OBJECT CREATION:
	Button ButtonObject(pin, switch normal state)
	
*/

#ifndef uButton_h
	#define uButton_h

	#include <arduino.h>

	class uButton
	{
		public:
			uButton(int pin, int normState);
			
			bool check();


		private:

			int pin;
			bool stateLast, switchDown, state, notRun;
	};

#endif