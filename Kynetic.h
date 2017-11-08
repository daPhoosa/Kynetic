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
void setPins()
{
   pinMode( X_MAX_ENDSTOP_PIN, INPUT_PULLUP );
   pinMode( Y_MAX_ENDSTOP_PIN, INPUT_PULLUP );
   pinMode( Z_MAX_ENDSTOP_PIN, INPUT_PULLUP );

   pinMode( X_MIN_ENDSTOP_PIN, INPUT_PULLUP );
   pinMode( Y_MIN_ENDSTOP_PIN, INPUT_PULLUP );
   pinMode( Z_MIN_ENDSTOP_PIN, INPUT_PULLUP );

   pinMode( SELECT_BUTTON_PIN, INPUT_PULLUP );
   pinMode( UP_BUTTON_PIN, INPUT_PULLUP );
   pinMode( DOWN_BUTTON_PIN, INPUT_PULLUP );
}





// **** OTHER FUNCTIONS ****

void motorController()
{
   static float x, y, z, a, b, c;

   motion.getTargetLocation( x, y, z );
   
   machine.invKinematics( x, y, z, a, b, c );
   
   A_motor.setSpeed( MOTION_CONTROL_HZ * ( a - A_motor.getPositionMM() ));
   B_motor.setSpeed( MOTION_CONTROL_HZ * ( b - B_motor.getPositionMM() ));
   C_motor.setSpeed( MOTION_CONTROL_HZ * ( c - C_motor.getPositionMM() ));
   
}