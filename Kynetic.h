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

#include <uButton.h>

// **** GLOBAL VARIABLES ****
bool homePositionSet = false;
bool runProgram = false;
bool getNextProgramBlock = false;
bool executeNextBlock = false;
bool fileComplete = true;
bool delayedExecute = false; // use this to force the movement buffer to empty before doing some operation



// **** BUTTONS ****
uButton SelectBtn(SELECT_BUTTON_PIN, SELECT_BUTTON_PRESSED);
uButton UpBtn(    UP_BUTTON_PIN,     UP_BUTTON_PRESSED);
uButton DnBtn(    DOWN_BUTTON_PIN,   DOWN_BUTTON_PRESSED);




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

   if( motion.getSpeed() > MACHINE_VEL_STEP )
   {
      A_motor.setSpeed( MOTION_CONTROL_HZ * ( motor.x - A_motor.getPositionMM() ));
      B_motor.setSpeed( MOTION_CONTROL_HZ * ( motor.y - B_motor.getPositionMM() ));
      C_motor.setSpeed( MOTION_CONTROL_HZ * ( motor.z - C_motor.getPositionMM() ));
   }
   else // force stop to avoid stepper "chatter"
   {
      A_motor.setSpeed( 0.0f );
      B_motor.setSpeed( 0.0f );
      C_motor.setSpeed( 0.0f );
   }

}


void criticalMotionControl()
{
   if( runProgram )
   {
      motorController();
      motionControl.collectStats();
   }
   else
   {
      if( machine.executeHome() )
      {
         Vec3 cart;

         machine.fwdKinematics( A_motor.getPositionMM(), B_motor.getPositionMM(), C_motor.getPositionMM(), cart.x, cart.y, cart.z ); // compute current cartesian start location

         //display(cart);

         runProgram = true;

         gCodeSetPosition( cart.x, cart.y, cart.z, 0.0f );

         motion.startMoving( cart.x, cart.y, cart.z );

         startPollTimers();
      }
   }
         
}


void blockFeeder()
{
   if( runProgram && motion.bufferVacancy() )
   {
      executeCode();
      getNextProgramBlock = true; // don't get the next program line until this one has been handed to the motion controller
      //Serial.println("*");
   }
}


programReader()
{
   if( !readNextProgramLine() )
   {
      fileComplete = true;
      //Serial.println("1");
   }
   else
   {
      //Serial.println("0");
   }
   getNextProgramBlock = false;   
}


void buttons()
{
   if( SelectBtn.check() )
   {
      if( runProgram )
      {
         runProgram = false;
      }
      else
      {
         machine.startHome( true, true, true );
         restartSD();
      }
   }
}


void display()
{

}