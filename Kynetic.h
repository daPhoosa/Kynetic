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
   pinMode( X_ENDSTOP_PIN, INPUT_PULLUP );
   pinMode( Y_ENDSTOP_PIN, INPUT_PULLUP );
   pinMode( Z_ENDSTOP_PIN, INPUT_PULLUP );

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

   float eLoc = motion.getExtrudeLocationMM();
   float eMot = D_motor.getPositionMM();

   float extrudeDelta = eLoc - eMot ;
   if( abs(extrudeDelta) > (1.5f / float(D_MOTOR_STEP_PER_MM)) ) // error must be more than 1 step, or motor is stopped
   {  
      float speed = float(MOTION_CONTROL_HZ) * extrudeDelta;
      D_motor.setSpeed( speed );
      
      //Serial.print(eLoc);Serial.print("\t");
      //Serial.print(eMot);Serial.print("\t");
      //Serial.println(speed);
   }
   else
   {
      D_motor.setSpeed( 0 );
   }
}


void motionRunner()
{
   if( KORE.runProgram && machine.allHomeCompleted() ) // Normal operation
   {
      motorController();
   }
   else if( machine.executeHome() )  // Home operation
   {
      Vec3 cart;

      machine.fwdKinematics( A_motor.getPositionMM(), B_motor.getPositionMM(), C_motor.getPositionMM(), cart.x, cart.y, cart.z ); // compute current cartesian start location

      gCodeSetPosition(   cart.x, cart.y, cart.z );
      motion.setPosition( cart.x, cart.y, cart.z );

      startPollTimers();

      KORE.runProgram = true;

      motionControl.resetStats();
      
      motion.startMoving();

      Serial.println("Home Complete");
   }
   else
   {
      //Serial.println("1");
   }

}


void blockFeeder()
{
   if( KORE.runProgram && motion.bufferVacancy() )
   {
      if( KORE.delayedExecute ) 
      {
         //Serial.print("Block Count: ");Serial.println(motion.getBlockCount());
         if( motion.blockQueueComplete() ) // don't execute delayed code until all queued moves are complete
         {
            //Serial.println("delayed execute!");
            executeCodeDelayed();
         }
         else
         {
            //motion.getBlockCount();
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

         Serial.println("START");
         KORE.manualPauseActive = false;
         restartSD();
         
         KORE.runProgram = true;
         
      }
   }
}


void displayDriver()
{

}


