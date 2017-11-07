/*
   Kynetic CNC Control Software
   Copyright (C) 2017 Phillip Schmidt

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



// **** GLOBAL VARIABLES ****
bool homePositionSet = false;
bool runProgram = false;
bool getNextProgramBlock = false;
bool executeNextBlock = false;






// **** STARTUP FUNCTIONS ****






// **** OTHER FUNCTIONS ****

void motorController()
{
   float x, y, z, a, b, c;

   motion.getTargetLocation( x, y, z );
   
   machine.invKinematics( x, y, z, a, b, c );
   
   A_motor.setSpeed( motionSmoothingRate * ( a - A_motor.getPositionMM() ));
   B_motor.setSpeed( motionSmoothingRate * ( b - B_motor.getPositionMM() ));
   C_motor.setSpeed( motionSmoothingRate * ( c - C_motor.getPositionMM() ));
   
}