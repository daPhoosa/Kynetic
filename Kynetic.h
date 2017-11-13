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
bool fileComplete = true;








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
   pinMode( UP_BUTTON_PIN,     INPUT_PULLUP );
   pinMode( DOWN_BUTTON_PIN,   INPUT_PULLUP );
}





// **** OTHER FUNCTIONS ****

void motorController()
{
   Vec3 cart, motor;

   motion.getTargetLocation( cart.x, cart.y, cart.z );
   
   machine.invKinematics( cart.x, cart.y, cart.z, motor.x, motor.y, motor.z );

   float A_delta, B_delta, C_delta;
   A_delta = motor.x - A_motor.getPositionMM();
   B_delta = motor.y - B_motor.getPositionMM();
   C_delta = motor.z - C_motor.getPositionMM();

   if( abs(A_delta) < EXACT_STOP_TOL ) A_delta = 0.0f;
   if( abs(B_delta) < EXACT_STOP_TOL ) B_delta = 0.0f;
   if( abs(C_delta) < EXACT_STOP_TOL ) C_delta = 0.0f;

   A_motor.setSpeed( MOTION_CONTROL_HZ * ( A_delta ));
   B_motor.setSpeed( MOTION_CONTROL_HZ * ( B_delta ));
   C_motor.setSpeed( MOTION_CONTROL_HZ * ( C_delta ));
}