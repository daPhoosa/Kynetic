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


float EPCOS_100k_47K_8304(int reading) // #1 -- EPCOS 100k with 4.7k pull-up (#8304)
{
   static const int sense = 0;
   static const int temp  = 1;
   static const int16_t table[][2] = { {183 * OVER_SAMPLE_CNT, 300},
                                       {195 * OVER_SAMPLE_CNT, 295},
                                       {209 * OVER_SAMPLE_CNT, 290},
                                       {224 * OVER_SAMPLE_CNT, 285},
                                       {241 * OVER_SAMPLE_CNT, 280},
                                       {259 * OVER_SAMPLE_CNT, 275},
                                       {278 * OVER_SAMPLE_CNT, 270},
                                       {299 * OVER_SAMPLE_CNT, 265},
                                       {323 * OVER_SAMPLE_CNT, 260},
                                       {348 * OVER_SAMPLE_CNT, 255},
                                       {376 * OVER_SAMPLE_CNT, 250},
                                       {407 * OVER_SAMPLE_CNT, 245},
                                       {440 * OVER_SAMPLE_CNT, 240},
                                       {477 * OVER_SAMPLE_CNT, 235},
                                       {518 * OVER_SAMPLE_CNT, 230},
                                       {562 * OVER_SAMPLE_CNT, 225},
                                       {611 * OVER_SAMPLE_CNT, 220},
                                       {665 * OVER_SAMPLE_CNT, 215},
                                       {725 * OVER_SAMPLE_CNT, 210},
                                       {790 * OVER_SAMPLE_CNT, 205},
                                       {862 * OVER_SAMPLE_CNT, 200},
                                       {942 * OVER_SAMPLE_CNT, 195},
                                       {1030 * OVER_SAMPLE_CNT, 190},
                                       {1126 * OVER_SAMPLE_CNT, 185},
                                       {1232 * OVER_SAMPLE_CNT, 180},
                                       {1349 * OVER_SAMPLE_CNT, 175},
                                       {1477 * OVER_SAMPLE_CNT, 170},
                                       {1618 * OVER_SAMPLE_CNT, 165},
                                       {1771 * OVER_SAMPLE_CNT, 160},
                                       {1939 * OVER_SAMPLE_CNT, 155},
                                       {2121 * OVER_SAMPLE_CNT, 150},
                                       {2318 * OVER_SAMPLE_CNT, 145},
                                       {2531 * OVER_SAMPLE_CNT, 140},
                                       {2758 * OVER_SAMPLE_CNT, 135},
                                       {3002 * OVER_SAMPLE_CNT, 130},
                                       {3259 * OVER_SAMPLE_CNT, 125},
                                       {3530 * OVER_SAMPLE_CNT, 120},
                                       {3812 * OVER_SAMPLE_CNT, 115},
                                       {4103 * OVER_SAMPLE_CNT, 110},
                                       {4402 * OVER_SAMPLE_CNT, 105},
                                       {4704 * OVER_SAMPLE_CNT, 100},
                                       {5006 * OVER_SAMPLE_CNT, 95},
                                       {5306 * OVER_SAMPLE_CNT, 90},
                                       {5599 * OVER_SAMPLE_CNT, 85},
                                       {5881 * OVER_SAMPLE_CNT, 80},
                                       {6151 * OVER_SAMPLE_CNT, 75},
                                       {6405 * OVER_SAMPLE_CNT, 70},
                                       {6640 * OVER_SAMPLE_CNT, 65},
                                       {6857 * OVER_SAMPLE_CNT, 60},
                                       {7053 * OVER_SAMPLE_CNT, 55},
                                       {7229 * OVER_SAMPLE_CNT, 50},
                                       {7384 * OVER_SAMPLE_CNT, 45},
                                       {7520 * OVER_SAMPLE_CNT, 40},
                                       {7638 * OVER_SAMPLE_CNT, 35},
                                       {7739 * OVER_SAMPLE_CNT, 30},
                                       {7824 * OVER_SAMPLE_CNT, 25},
                                       {7896 * OVER_SAMPLE_CNT, 20},
                                       {7956 * OVER_SAMPLE_CNT, 15},
                                       {8005 * OVER_SAMPLE_CNT, 10},
                                       {8045 * OVER_SAMPLE_CNT, 5},
                                       {8078 * OVER_SAMPLE_CNT, 0},
                                       {8104 * OVER_SAMPLE_CNT, -5},
                                       {8125 * OVER_SAMPLE_CNT, -10},
                                       {8141 * OVER_SAMPLE_CNT, -15},
                                       {8154 * OVER_SAMPLE_CNT, -20},
                                       {8163 * OVER_SAMPLE_CNT, -25},
                                       {8171 * OVER_SAMPLE_CNT, -30},
                                       {8177 * OVER_SAMPLE_CNT, -35},
                                       {8181 * OVER_SAMPLE_CNT, -40},
                                       {8184 * OVER_SAMPLE_CNT, -45},
                                       {8186 * OVER_SAMPLE_CNT, -50},
                                       {8188 * OVER_SAMPLE_CNT, -55}};

   static const int maxIndex = sizeof(table) / ( 2 * sizeof(table[0][0]) ) - 1;

   int lowIndex = 1;
   while( table[lowIndex][sense] < reading && lowIndex < maxIndex) lowIndex++;

   int highIndex = lowIndex - 1;

   float slope = float( table[lowIndex][temp] - table[highIndex][temp] ) / float( table[lowIndex][sense] - table[highIndex][sense] );

   float temperature = float(table[highIndex][temp]) + slope * float( reading - table[highIndex][sense] );

   return temperature;
}


float ATC_GT1042_100k_47K(int reading)     // #5 -- ATC GT104-2 100k with 4.7k pull-up
{
   static const int sense = 0;
   static const int temp  = 1;
   static const int16_t table[][2] = { {138 * OVER_SAMPLE_CNT, 300}, 
                                       {159 * OVER_SAMPLE_CNT, 290},
                                       {184 * OVER_SAMPLE_CNT, 280},
                                       {214 * OVER_SAMPLE_CNT, 270},
                                       {249 * OVER_SAMPLE_CNT, 260},
                                       {292 * OVER_SAMPLE_CNT, 250},
                                       {345 * OVER_SAMPLE_CNT, 240},
                                       {409 * OVER_SAMPLE_CNT, 230},
                                       {487 * OVER_SAMPLE_CNT, 220},
                                       {582 * OVER_SAMPLE_CNT, 210},
                                       {700 * OVER_SAMPLE_CNT, 200},
                                       {845 * OVER_SAMPLE_CNT, 190},
                                       {1024 * OVER_SAMPLE_CNT, 180},
                                       {1244 * OVER_SAMPLE_CNT, 170},
                                       {1512 * OVER_SAMPLE_CNT, 160},
                                       {1838 * OVER_SAMPLE_CNT, 150},
                                       {2230 * OVER_SAMPLE_CNT, 140},
                                       {2690 * OVER_SAMPLE_CNT, 130},
                                       {3219 * OVER_SAMPLE_CNT, 120},
                                       {3808 * OVER_SAMPLE_CNT, 110},
                                       {4438 * OVER_SAMPLE_CNT, 100},
                                       {5083 * OVER_SAMPLE_CNT, 90},
                                       {5708 * OVER_SAMPLE_CNT, 80},
                                       {6280 * OVER_SAMPLE_CNT, 70},
                                       {6777 * OVER_SAMPLE_CNT, 60},
                                       {7184 * OVER_SAMPLE_CNT, 50},
                                       {7500 * OVER_SAMPLE_CNT, 40},
                                       {7734 * OVER_SAMPLE_CNT, 30},
                                       {7899 * OVER_SAMPLE_CNT, 20},
                                       {8011 * OVER_SAMPLE_CNT, 10},
                                       {8085 * OVER_SAMPLE_CNT, 0},
                                       {8130 * OVER_SAMPLE_CNT, -10},
                                       {8158 * OVER_SAMPLE_CNT, -20},
                                       {8174 * OVER_SAMPLE_CNT, -30},
                                       {8183 * OVER_SAMPLE_CNT, -40},
                                       {8188 * OVER_SAMPLE_CNT, -50}};

   static const int maxIndex = sizeof(table) / ( 2 * sizeof(table[0][0]) ) - 1;

   int lowIndex = 1;
   while( table[lowIndex][sense] < reading && lowIndex < maxIndex) lowIndex++;

   int highIndex = lowIndex - 1;

   float slope = float( table[lowIndex][temp] - table[highIndex][temp] ) / float( table[lowIndex][sense] - table[highIndex][sense] );

   float temperature = float(table[highIndex][temp]) + slope * float( reading - table[highIndex][sense] );

   return temperature;
}
