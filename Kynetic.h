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
//bool getNextProgramBlock = false;
//bool executeNextBlock = false;
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

void abortAll()
{
   display("ABORT ALL EXECUTED \n");

   motion.abortMotion();
   stopMotors();
   
   KORE.runProgram = false;
   KORE.manualPauseActive = false;

   KORE.bedTargetTemp = 0;    // turn heaters off
   KORE.extrude1TargetTemp = 0;
   heaterOperator();
   heaterPWM();
}


void motorController()
{
   // moved to ISR
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

      //motionControl.resetStats();
      //blockRead.resetStats();
      
      motion.startMoving();

      display(String("Home Complete \n"));
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
         display( "Extruder to Temp: " + String(KORE.extrude1Temp, 1) + "  Time:" );
         display( (millis() - extStartTime) / 1000 );
         display("\n");
         KORE.programStartTime = millis();
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
         display( "Bed to Temp: " + String(KORE.bedTemp, 1) + "  Time:" );
         display( (millis() - bedStartTime) / 1000 );
         display("\n");
         KORE.programStartTime = millis();
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
            display("File Complete \n");
            abortAll();

            //blockRead.displayStats();

            uint32_t runTime = (millis() - KORE.programStartTime) / 1000;  // time in seconds
            uint32_t t = runTime / 3600;
            display( "H:" ); display( t );
            runTime = runTime % 3600;
            t = runTime / 60;
            display( " M:" ); display( t );
            runTime = runTime % 60;
            display( " S:" ); display( runTime ); display( "\n" );
         }
      }

      //getNextProgramBlock = false;   
   }
}


bool codeReader()
{
   if( KORE.runProgram && motion.bufferVacancy() )
   {
      if( KORE.delayedExecute ) 
      {
         if( motion.blockQueueComplete() ) // don't execute delayed code until all queued moves are complete
         {
            //Serial.println("delayed execute!");
            executeCodeDelayed();
         }
      }
      else  
      {
         executeCodeNow();
         programReader();
         //getNextProgramBlock = true; // don't get the next program line until this one has been handed to the motion controller
         return true;
      }
   }
   return false;
}


void buttonWatcher()
{
   if( SelectBtn.check() )
   {
      if( KORE.runProgram )
      {
         KORE.manualPauseActive = !KORE.manualPauseActive;
         if( KORE.manualPauseActive )
         {
            display("PAUSE\n");
         }
         else
         {
            display("RESUME\n");
         }
      }
      else
      {
         display("START\n");
         
         restartSD();
         
         KORE.manualPauseActive = false;
         fileComplete = false;
         KORE.runProgram = true;

         KORE.programStartTime = millis();
      }
   }
}


void displayDriver()
{

}


void watchDogChecks()
{
   if( KORE.heaterWatchDog++ > MOTION_CONTROL_HZ )
   {
      if( KORE.bedTargetTemp > 0 || KORE.extrude1TargetTemp > 0 )
      {
         display("HEATER ERROR - CONTROL LOOP FAILED TO EXECUTE\n");
         abortAll();
      }
      KORE.heaterWatchDog = 0;
   }
}


