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

#include "config.h"
#include "config_adv.h"
#include "Kynetic_pins.h"

#include "motors.h"
#include "dataStreams.h"
#include "timers.h"
#include "motion.h"
#include "gCode.h"
#include "temperatureSensor.h"
#include "heaters.h"

#include "3DMath.h"
#include "Machines\cartesian.h"
#include "Machines\coreXY.h"
#include "Machines\delta.h"

#include "kynetic.h"


void setup() 
{

   startSerial();

   startSD();

   startStepperTickISR();

   setPins();

   armMotors();
  
   startPollTimers();
}


void loop() 
{
   
   // Nested if-else priority scheme
   //  * After any operation completes, higher priority operations are given the first opportunity to run.
   //  * All operations should run quickly so that higher priority operations are not delayed excessively.
   
   if( motionControl.check() )  // Highest Priority
   {
      motionRunner();
   }
   else if( motionControl.precheck(10) ) // prevent executing other code if very close to next motion control operation
   {
      // timeWaster  --  do nothing
   }
   else if( blockExecute.check() ) // Execute G code, feed blocks to the motion controller 
   {
      blockFeeder();
   }
   else if( getNextProgramBlock ) // Read SD card and Parse G code
   {
      programReader();
   }
   else if( heaterManager.check() )
   {
      heaterOperator();     // operate heaters
   }
   else if ( buttonsAndUI.check() ) // check if any buttons are depressed and update Display
   {
      buttonWatcher();
      displayDriver();
   }
   else if( maintenance.check() ) // Lowest Priority
   {
      setMotorTickRate();
      //motionControl.displayStats();

      Serial.print( Extruder1Temp );
      Serial.print("   ");
      Serial.print( 20 );
      Serial.print("   ");
      Serial.println( 100 );
   }   

}




