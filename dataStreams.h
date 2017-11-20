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

#include <SdFat.h>               // https://github.com/greiman/SdFat                --instal to libraries diectory


// **** SD CARD INTERFACE ****
SdFatSdioEX sdEx;
File file;

bool endOfFileFound;


void startSerial()
{
   #ifdef SERIAL_PORT
      SERIAL_PORT.begin(250000);
      //while(!Serial){}
   #endif
}


void startSD()
{
   if( sdEx.begin() )
   {
      file = sdEx.open("print.nc", O_READ);
      if(!file)
      {
         SERIAL_PORT.println("Open File Failed!");
      }   
   }
   else
   {
      SERIAL_PORT.println("Start SD Failed!");
   }
}


void restartSD()
{
      /*
   if( file )
   {
      file.close(); // close if the file is already open
   }
   */
  
   startSD();
}