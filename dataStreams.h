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


// **** SD CARD INTERFACE ****
SdFatSdioEX SD;
File file;

bool endOfFileFound;


void display( const String& msg )
{
   #ifdef SERIAL_PORT
      if(SERIAL_PORT)
      {
         SERIAL_PORT.print(msg);
      }
   #endif
}


void display( const int& msg )
{
   #ifdef SERIAL_PORT
      if(SERIAL_PORT)
      {
         SERIAL_PORT.print(msg);
      }
   #endif
}


void startSerial()
{
   #ifdef SERIAL_PORT
      SERIAL_PORT.begin(250000);
   #endif
}


void startSD()
{
   if( file.isOpen() ) 
   {
      file.close();
   }

   if( SD.begin() )
   {
      file = SD.open("print.gcode", O_READ);
      if(!file)
      {
         display("Open File Failed! \n");
         KORE.runProgram = false;
      }   
   }
   else
   {
      display("Start SD Failed! \n");
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