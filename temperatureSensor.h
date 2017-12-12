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
   const int size = 72;
   const int table[][2] = {{8188,-55},  // EPCOS 100k with 4.7k pull-up
                           {8186,-50},
                           {8184,-45},
                           {8181,-40},
                           {8177,-35},
                           {8171,-30},
                           {8163,-25},
                           {8154,-20},
                           {8141,-15},
                           {8125,-10},
                           {8104,-5},
                           {8078,0},
                           {8045,5},
                           {8005,10},
                           {7956,15},
                           {7896,20},
                           {7824,25},
                           {7739,30},
                           {7638,35},
                           {7520,40},
                           {7384,45},
                           {7229,50},
                           {7053,55},
                           {6857,60},
                           {6640,65},
                           {6405,70},
                           {6151,75},
                           {5881,80},
                           {5599,85},
                           {5306,90},
                           {5006,95},
                           {4704,100},
                           {4402,105},
                           {4103,110},
                           {3812,115},
                           {3530,120},
                           {3259,125},
                           {3002,130},
                           {2758,135},
                           {2531,140},
                           {2318,145},
                           {2121,150},
                           {1939,155},
                           {1771,160},
                           {1618,165},
                           {1477,170},
                           {1349,175},
                           {1232,180},
                           {1126,185},
                           {1030,190},
                           {942,195},
                           {862,200},
                           {790,205},
                           {725,210},
                           {665,215},
                           {611,220},
                           {562,225},
                           {518,230},
                           {477,235},
                           {440,240},
                           {407,245},
                           {376,250},
                           {348,255},
                           {323,260},
                           {299,265},
                           {278,270},
                           {259,275},
                           {241,280},
                           {224,285},
                           {209,290},
                           {195,295},
                           {183,300}};

   return 0.0f;
}

