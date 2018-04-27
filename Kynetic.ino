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

#include <SdFat.h>               // https://github.com/greiman/SdFat                  --instal to libraries diectory
#include <FrequencyTimer2.h>     // https://github.com/PaulStoffregen/FrequencyTimer2 --included with teensyduino
#include "src/uButton/uButton.h"
#include "src/SmoothMove/SmoothMove.h"
#include "src/MedianFilter/MedianFilter.h"
#include "src/PollTimer/PollTimer.h"

#include "src/stepperMotor/stepperMotor.h"
#include "motors.h"

#include "dataStreams.h"
#include "timers.h"
#include "temperatureSensor.h"
#include "heaters.h"

#include "3DMath.h"
#include "src/Machines/cartesian.h"
#include "src/Machines/coreXY.h"
#include "src/Machines/delta.h"

#include "gCodeStructure.h"
#include "motion.h"
#include "gCode.h"

#include "kynetic.h"


void setup() 
{
   startSerial();

   startSD();

   startStepperTickISR();

   setupPins();

   stopMotors();

   configMotion();
  
   startPollTimers();

   display( "\nKYNETIC CNC CONTROLLER\n" );
}


void loop() 
{
   
   if( motionControl.check() )  // Highest Priority
   {
      watchDogChecks();
   }
   else if( blockExecute.check() ) // Execute G code, feed blocks to the motion controller 
   {
      if( codeReader() )
      {
         //blockExecute.collectStats();
      }
   }
   else if( softPWM.check() )
   {
      heaterPWM();      // modulate heater power
   }
   else if( heaterManager.check() )
   {
      heaterOperator(); // operate heaters

      if( heaterSafetyChecks() ) abortAll();
   }
   else if ( buttonsAndUI.check() ) // check if any buttons are depressed and update Display
   {
      buttonWatcher();
      displayDriver();
   }
   else if( maintenance.check() ) // Lowest Priority
   {
      //Serial.println(stepperTickCount);
      setMotorTickRate();

      //Serial.println(KORE.runProgram);

      //extruder1_PID.display();      // Monitor extrude PID values
      //bed_PID.display();            // Monitor bed PID values

      if( KORE.runProgram )
      {
         //Serial.println( float(funCounter*100) / 1000000.0f, 1);  // interupt CPU usage
         /*
         if( funCounter )
         {
            display( String(funCounter) );
         //motionControl.displayStats();
         } 
         */

         //display(funCounter);display("\n");
         funCounter = 0;

         //Serial.print(motion.currentBlockIndex); Serial.print("\t"); Serial.println(motion.moveBuffer[motion.currentBlockIndex].staticExtrude);

         //display(A_motor.getSpeed());display(" ");display(B_motor.getSpeed());display(" ");display(C_motor.getSpeed());display("\n");
         
         //display( motion.getBlockCount() );display("\n");

         //motionControl.displayStats();
         //blockExecute.displayStats();
         //blockRead.displayStats();

         //blockRead.resetStats();
         //Serial.println( motionControl.getPctCPU() + 9.6f, 1);

         //display(extrude1Filter.getStDev());display(" ");display(bedFilter.getStDev());display("\n");
         
         //Serial.print(KORE.bedTemp, 2);Serial.print("   ");Serial.println(KORE.extrude1Temp, 2);

         //Serial.print(KORE.extrude1TargetTemp);Serial.print("   ");Serial.println(KORE.extrude1Temp, 2);

         //Serial.println(machine.allHomeCompleted());
         //Serial.print(motion.getExtrudeLocationMM());Serial.print("   ");Serial.println(D_motor.getPositionMM());
      }
   }   
 
}