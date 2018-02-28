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


#include "slowPWM.h"
#include "heaterPID.h"

heaterPID extruder1_PID( HEATER_MANAGER_HZ, EXTRUDER1_PID );
slowPWM   extruder1_PWM( MIN_HEATER_PERIOD );

heaterPID bed_PID( HEATER_MANAGER_HZ, BED_HEATER_PID );
slowPWM   bed_PWM( MIN_HEATER_PERIOD );


void heaterPWM()  // toogle heaters to get desired duty cycle
{
   if( extruder1_PWM.check() && KORE.extrude1TargetTemp )
   {
      digitalWrite(EXTRUDER1_PWM_PIN, HIGH);
   }
   else
   {
      digitalWrite(EXTRUDER1_PWM_PIN, LOW);
   }

   if( bed_PWM.check() && KORE.bedTargetTemp )
   {
      digitalWrite(BED_HEATER_PWM_PIN, HIGH);
   }
   else
   {
      digitalWrite(BED_HEATER_PWM_PIN, LOW);
   }
}


bool heaterSafetyChecks()
{
   bool alarm = false;

   // broken wire on thermistor check
   if( KORE.extrude1Temp < 0 ) 
   {
      display("ALARM: EXTRUDER 1 THERMISTOR CONNECTION FAILURE \n");
      alarm = true;
   }

   if( KORE.bedTemp < 0 )      
   {
      display("ALARM: BED THERMISTOR CONNECTION FAILURE \n");
      alarm = true;
   }

   if( KORE.extrude1TargetTemp )
   {
      if( extruder1_PID.getSaturationTime() > MAX_EXT1_HEAT_TIME )   // thermistor not touching, bad heating element
      {
         display("ALARM: EXTRUDER 1 EXCESSIVE HEAT TIME \n");
         alarm = true;
      }

      int reading = extrude1Filter.getStDev();
      if( reading > MAX_ERATIC_EXT_THERM )
      {
         display("ALARM: EXTRUDER 1 THERMISTOR ERATIC READINGS ");
         display(reading);
         display("\n");
         alarm = true;
      }
   }
   
   if( KORE.bedTargetTemp )
   {
      if( bed_PID.getSaturationTime() > MAX_BED_HEAT_TIME )
      {
         display("ALARM: BED EXCESSIVE HEAT TIME \n");
         alarm = true;
      }

      int reading = bedFilter.getStDev();
      if( reading > MAX_ERATIC_BED_THERM )
      {
         display("ALARM: BED THERMISTOR ERATIC READINGS ");
         display(reading);
         display("\n");
         alarm = true;
      }
   }

   return alarm;
}


void heaterOperator()  // operate heaters
{
   // do cool things... ha ha
   KORE.extrude1Temp = getExtruder1Temp();
   KORE.bedTemp = getBedTemp();

   extruder1_PWM.set( extruder1_PID.in( KORE.extrude1TargetTemp, KORE.extrude1Temp ));
   bed_PWM.set( bed_PID.in( KORE.bedTargetTemp, KORE.bedTemp ));

   //extruder1_PID.display();
   //bed_PID.display();

   KORE.heaterWatchDog = 0; // reset counter
}

