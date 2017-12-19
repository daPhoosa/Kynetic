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


#include "thermistorTables.h"
#include <MedianFilter.h>

MedianFilter extrude1Filter( 9, 1000);
MedianFilter bedFilter( 9, 1000);


float tempConvert( int type, int reading )
{
   switch(type)
   {
      case 1 :
        return EPCOS_100k_47K_8304(reading);

      case 5 :
         return ATC_GT1042_100k_47K(reading);

      default :
         return 999.9f;
   }
}


float getExtruder1Temp()
{
   static uint16_t tempList[OVER_SAMPLE_CNT];
   static int listIndex = 0;

   int sensorReading = extrude1Filter.in( analogRead(EXTRUDER1_THERMISTOR) );

   tempList[listIndex] = sensorReading;
   listIndex++;
   if( listIndex == OVER_SAMPLE_CNT) listIndex = 0;

   uint32_t sampleSum = 0;
   for( int i = 0; i < OVER_SAMPLE_CNT; i++ )
   {
      sampleSum += tempList[i];
   }

   return tempConvert( EXTRUDER1_SENSOR_TYPE, sampleSum );
}


float getBedTemp()
{
   static int tempList[OVER_SAMPLE_CNT];
   static int listIndex = 0;

   int sensorReading = bedFilter.in( analogRead(BED_THERMISTOR) );

   tempList[listIndex] = sensorReading;
   listIndex++;
   if( listIndex == OVER_SAMPLE_CNT) listIndex = 0;

   int sampleSum = 0;
   for( int i = 0; i < OVER_SAMPLE_CNT; i++ )
   {
      sampleSum += tempList[i];
   }

   return tempConvert( BED_SENSOR_TYPE, sampleSum );
}
