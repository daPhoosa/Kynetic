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
#include "temperatureSensor.h"
#include "heaters.h"

#include "3DMath.h"
#include "Machines\cartesian.h"
#include "Machines\coreXY.h"
#include "Machines\delta.h"

#include "gCode.h"

#include "kynetic.h"


void setup() 
{
   startSerial();

   startSD();

   startStepperTickISR();

   setPins();

   armMotors();

   configMotion();
  
   startPollTimers();
}


void loop() 
{
   
   if( motionControl.check() )  // Highest Priority
   {
      motionRunner();
      //funCounter++;

      //motionControl.collectStats();
      
      lowerPriorityOperations(); // only one of these will run each call

      motionControl.collectStats();
   }
 
}


void lowerPriorityOperations()
{
   // Nested if-else priority scheme
   //  * After any operation completes, higher priority operations are given the first opportunity to run.
   //  * All operations should run quickly so that higher priority operations are not delayed excessively.

   if( motionControl.precheck(10) ) // immedately return to motion control if last one was delayed
   {
      funCounter++;
   }
   else if( blockExecute.check() ) // Execute G code, feed blocks to the motion controller 
   {
      blockFeeder();
      //funCounter++;
   }
   else if( blockRead.check() && getNextProgramBlock ) // Read SD card and Parse G code
   {
      programReader();
   }
   else if( softPWM.check() )
   {
      heaterPWM();      // modulate heater power
   }
   else if( heaterManager.check() )
   {
      heaterOperator(); // operate heaters
   }
   else if ( buttonsAndUI.check() ) // check if any buttons are depressed and update Display
   {
      buttonWatcher();
      displayDriver();
   }
   else if( maintenance.check() ) // Lowest Priority
   {
      setMotorTickRate();

      //Serial.println( float(funCounter*100) / 1000000.0f, 1);  // interupt CPU usage
      if( funCounter )
      {
         Serial.println( funCounter );
         motionControl.displayStats();
      }
      funCounter = 0;

      motionControl.displayStats();
      //Serial.println( motionControl.getPctCPU() + 9.6f, 1);
      
      //Serial.print(KORE.bedTemp, 2);Serial.print("   ");Serial.println(KORE.extrude1Temp, 2);

      //Serial.println(machine.allHomeCompleted());
      //Serial.print(motion.getExtrudeLocationMM());Serial.print("   ");Serial.println(D_motor.getPositionMM());
   }   
}



