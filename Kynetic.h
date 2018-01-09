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
bool getNextProgramBlock = false;
bool executeNextBlock = false;
bool fileComplete = true;





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

   pinMode( BED_HEATER_PWM_PIN, OUTPUT );
   pinMode( EXTRUDER1_PWM_PIN,  OUTPUT );

   analogReadResolution(13);
}





// **** OTHER FUNCTIONS ****

void motorController()
{
   Vec3 cart, motor;

   motion.getTargetLocation( cart.x, cart.y, cart.z );
   
   machine.invKinematics( cart.x, cart.y, cart.z, motor.x, motor.y, motor.z );

   if( motion.getSpeed() > MACHINE_VEL_STEP )
   {
      A_motor.setSpeed( float(MOTION_CONTROL_HZ) * ( motor.x - A_motor.getPositionMM() ));
      B_motor.setSpeed( float(MOTION_CONTROL_HZ) * ( motor.y - B_motor.getPositionMM() ));
      C_motor.setSpeed( float(MOTION_CONTROL_HZ) * ( motor.z - C_motor.getPositionMM() ));
   }
   else // force stop to avoid stepper "chatter"
   {
      A_motor.setSpeed( 0 );
      B_motor.setSpeed( 0 );
      C_motor.setSpeed( 0 );
   }

   float extrudeDelta = motion.getExtrudeLocationMM() - D_motor.getPositionMM();
   if( abs(extrudeDelta) > (1.5f / float(D_MOTOR_STEP_PER_MM)) ) // error must be more than 1 step, or motor is stopped
   {
      D_motor.setSpeed( float(MOTION_CONTROL_HZ) * extrudeDelta );
   }
   else
   {
      D_motor.setSpeed( 0 );
   }
   

}


void motionRunner()
{
   if( KORE.runProgram && machine.allHomeCompleted() )
   {
      motorController();
      motionControl.collectStats();
   }
   else if( machine.executeHome() )
   {
      Vec3 cart;

      machine.fwdKinematics( A_motor.getPositionMM(), B_motor.getPositionMM(), C_motor.getPositionMM(), cart.x, cart.y, cart.z ); // compute current cartesian start location

      gCodeSetPosition( cart.x, cart.y, cart.z, 0.0f );

      motion.addLinear_Block( cart.x, cart.y, cart.z, 0.1 ); // needed?

      startPollTimers();

      KORE.runProgram = true;

      motion.setPosition( cart.x, cart.y, cart.z );
      motion.startMoving();

      //Serial.println("Home Complete");
   }
   else
   {
      motionControl.resetStats();
   }
}


void blockFeeder()
{
   if( KORE.runProgram && motion.bufferVacancy() )
   {
      if( KORE.delayedExecute ) 
      {
         //Serial.println(motion.blockQueueComplete());
         if( motion.blockQueueComplete() ) // don't execute delayed code until all queued moves are complete
         {
            Serial.println("delayed execute!");
            executeCodeDelayed();
         }
      }
      else  
      {
         executeCodeNow();
         getNextProgramBlock = true; // don't get the next program line until this one has been handed to the motion controller
      }
   }
}


bool pauseManager() // return true if pause is active
{
   bool pauseStatus = KORE.manualPauseActive;

   if( KORE.extrude1_wait ) 
   {
      if( float(KORE.extrude1TargetTemp) - KORE.extrude1Temp < 1.0f )
      {
         KORE.extrude1_wait = false; // up to temp
         //Serial.print("Extruder to Temp: ");Serial.println(KORE.extrude1Temp, 1);
      }
      else
      {
         pauseStatus = true;
      }
   }

   if( KORE.bed_wait ) 
   {
      if( float(KORE.bedTargetTemp) - KORE.bedTemp < 1.0f )
      {
         KORE.bed_wait = false; // up to temp
         //Serial.print("Bed to Temp: ");Serial.println(KORE.bedTemp, 1);
      }
      else
      {
         pauseStatus = true;
      }
   }

   return pauseStatus;
}


void programReader()
{
   if( !pauseManager() )
   {
      if( !readNextProgramLine() )
      {
         fileComplete = true;
         if( motion.blockQueueComplete() && !KORE.delayedExecute )
         {
            KORE.runProgram = false;
         }
         //Serial.println("1");
      }
      else
      {
         //Serial.println("0");
      }
      getNextProgramBlock = false;   
   }
}


void buttonWatcher()
{
   if( SelectBtn.check() )
   {
      //Serial.print("SELECT BUTTON - ");
      if( KORE.runProgram )
      {
         KORE.manualPauseActive = !KORE.manualPauseActive;
      }
      else
      {
         //Serial.println("START");
         KORE.manualPauseActive = false;
         //machine.startHome( true, true, true );
         restartSD();
         //getNextProgramBlock = true;
         
         KORE.runProgram = true;
      }
   }
}


void displayDriver()
{

}


