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


class tempSensor
{
   public:

      tempSensor( int periodMS );

      float check( int ADC_reading );


   private:



};


tempSensor::tempSensor( int sensorType )
{

}

float tempConvert(int reading)
{
   const int sense = 0;
   const int temp  = 1;
   const int16_t table[][2] = {{183,300},     // EPCOS 100k with 4.7k pull-up
                              {195,295},
                              {209,290},
                              {224,285},
                              {241,280},
                              {259,275},
                              {278,270},
                              {299,265},
                              {323,260},
                              {348,255},
                              {376,250},
                              {407,245},
                              {440,240},
                              {477,235},
                              {518,230},
                              {562,225},
                              {611,220},
                              {665,215},
                              {725,210},
                              {790,205},
                              {862,200},
                              {942,195},
                              {1030,190},
                              {1126,185},
                              {1232,180},
                              {1349,175},
                              {1477,170},
                              {1618,165},
                              {1771,160},
                              {1939,155},
                              {2121,150},
                              {2318,145},
                              {2531,140},
                              {2758,135},
                              {3002,130},
                              {3259,125},
                              {3530,120},
                              {3812,115},
                              {4103,110},
                              {4402,105},
                              {4704,100},
                              {5006,95},
                              {5306,90},
                              {5599,85},
                              {5881,80},
                              {6151,75},
                              {6405,70},
                              {6640,65},
                              {6857,60},
                              {7053,55},
                              {7229,50},
                              {7384,45},
                              {7520,40},
                              {7638,35},
                              {7739,30},
                              {7824,25},
                              {7896,20},
                              {7956,15},
                              {8005,10},
                              {8045,5},
                              {8078,0},
                              {8104,-5},
                              {8125,-10},
                              {8141,-15},
                              {8154,-20},
                              {8163,-25},
                              {8171,-30},
                              {8177,-35},
                              {8181,-40},
                              {8184,-45},
                              {8186,-50},
                              {8188,-55}};

   int const maxIndex = sizeof(table) / ( 2 * sizeof(table[0][0]) ) - 1;
   int lowIndex = 1;
   while( table[lowIndex][sense] < reading && lowIndex < maxIndex) lowIndex++;

   int highIndex = lowIndex - 1;

   float slope = float( table[lowIndex][temp] - table[highIndex][temp] ) / float( table[lowIndex][sense] - table[highIndex][sense] );

   float temperature = table[highIndex][temp] + slope * float( reading - table[highIndex][sense] );

   return temperature;
}

