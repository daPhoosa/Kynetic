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


#include "gCodeStructure.h"
#include "gCodeOperations.h"


char getNextChar()
{
   static const int SD_BUFFER_SIZE = 2048;

   static struct read_buffer_sd_t
   {
      char data[SD_BUFFER_SIZE];
      int  dataCount = 0;
      bool dataReady = false;
   } buffer[2];

   static int dataIndex = 0;
   static int active    = 0;    // read from this buffer
   static int fill      = 1;    // fill this buffer

   if( !buffer[fill].dataReady ) // check for empty buffer
   {
      if( file.available() )
      {
         buffer[fill].dataCount = file.read( &buffer[fill].data , SD_BUFFER_SIZE );
         if( buffer[fill].dataCount > 0 ) buffer[fill].dataReady = true;
         if( !buffer[active].dataReady )  // insure read starts with this buffer if active is empty
         {
            active    =  fill;
            fill      = !fill;
            dataIndex = 0;
         }
      }
   }

   if( buffer[active].dataReady ) // check if active buffer has data
   {
      char ch = buffer[active].data[dataIndex]; // get character

      dataIndex++;

      if( dataIndex >= buffer[active].dataCount )  // out of data in current buffer
      {
         buffer[active].dataReady = false;
         fill      =  active;   // swap active buffers
         active    = !active;
         dataIndex = 0;
      }

      return ch;
   }
   else // no more data in buffers ( file end has been found or read error )
   {
      return 0;
   }
}


void setState( char letter, float number )
{

   // list most common letters first to avoid uneeded compares

   if( letter == 'X' )
   {
      if( gCode.G[6] == 20 ) number *= 25.4f; // convert to mm

      if( gCode.G[0] == 92 )  // set position
      {
         gCode.X = number;
         return;
      }

      if( gCode.G[0] == 28 )  // send Home
      {
         gCode.X = number;
         gCode.homeX = true;
         return;
      }

      if( gCode.G[3] == 90 ) // absolute
      {
         number += gCode.workOffsetX; // shift from program to machine coordinate
         if( abs( gCode.X - number ) > 0.0009f )
         {
            gCode.X = number;
            gCode.newAxisMove = true;
         }
      }
      else if( gCode.G[3] == 91 ) // incremental
      {
         if( abs(number) > 0.0009f )
         {
            gCode.X += number; 
            gCode.newAxisMove = true;
         }
      }

      return;
   }

   if( letter == 'Y' )
   {
      if( gCode.G[6] == 20 ) number *= 25.4f; // convert to mm

      if( gCode.G[0] == 92 )  // set position
      {
         gCode.Y = number;
         return;
      }

      if( gCode.G[0] == 28 )  // send Home
      {
         gCode.Y = number;
         gCode.homeY = true;
         return;
      }

      if( gCode.G[3] == 90 ) // absolute
      {
         number += gCode.workOffsetY; // shift from program to machine coordinate
         if( abs(gCode.Y - number) > 0.0009f )
         {
            gCode.Y = number;
            gCode.newAxisMove = true;
         }
      }
      else if( gCode.G[3] == 91 ) // incremental
      {
         if( abs(number) > 0.0009f )
         {
            gCode.Y += number; 
            gCode.newAxisMove = true;
         }
      }

      return;
   }

   if( letter == 'E' )
   {
      if( gCode.G[6] == 20 ) number *= 25.4f; // convert to mm
      
      if( gCode.G[0] == 92 )  // set position
      {
         gCode.E = number;
         return;
      }

      if( gCode.extrudeAbsoluteMode ) // absolute
      {
         if( abs(gCode.E - number) > 0.0009f )
         {
            gCode.E = number;
            gCode.newExtruderMove = true;
         }
      }
      else  // incremental
      {
         if( abs(number) > 0.0009f )
         {
            gCode.E += number; 
            gCode.newExtruderMove = true;
         }
      }
      return;
   }

   if( letter == 'F' )
   {
      if( gCode.G[6] == 21 ) // metric
      {
         gCode.F = number * ( 1.0f / 60.0f); // convert: mm/min ==> mm/s
      } 
      else if( gCode.G[6] == 20 ) // imperial
      {
         gCode.F = number * (25.4f / 60.0f); // convert: in/min ==> mm/s
      }
      return;
   }

   if( letter == 'Z' )
   {
      if( gCode.G[6] == 20 ) number *= 25.4f; // convert to mm

      if( gCode.G[0] == 92 )  // set position
      {
         gCode.Z = number;
         return;
      }

      if( gCode.G[0] == 28 )  // send Home
      {
         gCode.Z = number;
         gCode.homeZ = true;
         return;
      }

      if( gCode.G[3] == 90 ) // absolute
      {
         number += gCode.workOffsetZ; // shift from program to machine coordinate
         if( abs(gCode.Z - number) > 0.0009f )
         {
            gCode.Z = number;
            gCode.newAxisMove = true;
         }
      }
      else if( gCode.G[3] == 91 ) // incremental
      {
         if( abs(number) > 0.0009f )
         {
            gCode.Z += number; 
            gCode.newAxisMove = true;
         }
      }

      return;
   }

   if( letter == 'G' )
   {
      byte num = byte(number);

      switch( num )
      {
         case 4 :    // Group 0 -- dwell, exact stop, Home
         case 9 :
         case 28:
         case 29:
         case 92:    // set position
            KORE.delayedExecute = true;
            gCode.G[0] = num;
            break;

         case 0 :    // Group 1
         case 1 :
         case 2 :
         case 3 :
            gCode.G[1] = num;
            break;

         case 17:    // Group 2
         case 18:
         case 19:
            gCode.G[2] = num;
            break;        

         case 90:    // Group 3
         case 91:
            gCode.G[3] = num;
            if( num == 90 ) gCode.extrudeAbsoluteMode = true;
            if( num == 91 ) gCode.extrudeAbsoluteMode = false;
            break;

                  // No Group 4

         case 93:    // Group 5
         case 94:
         case 95:
            gCode.G[5] = num;
            break;

         case 20:    // Group 6
         case 21:
            gCode.G[6] = num;
            break;

         case 40:    // Group 7
         case 41:
         case 42:
            gCode.G[7] = num;
            break;

         case 54:    // Group 12 -- work offsets
         case 55:
         case 56:
         case 57:
         case 58:
         case 59:
            gCode.G[12] = num;
            num -= 54;
            gCode.workOffsetX = gCode.WO[num].X;
            gCode.workOffsetY = gCode.WO[num].Y;
            gCode.workOffsetZ = gCode.WO[num].Z;
            break;


         default:
            break;
      }
      return;
   }

   if( letter == 'M' )
   {
      int num = int(number);

      gCode.M = num;
      gCode.newMcode = true;
      KORE.delayedExecute = true;
      return;
   }

   if( letter == 'P' )
   {
      gCode.P = number;
      return;
   }
   
   if( letter == 'I' )
   {
      if( gCode.G[1] == 2 || gCode.G[1] == 3 ) gCode.newAxisMove = true; // only flag new move if doing arc move
      gCode.I = number;
      return;
   }
   
   if( letter == 'J' )
   {
      if( gCode.G[1] == 2 || gCode.G[1] == 3 ) gCode.newAxisMove = true; // only flag new move if doing arc move
      gCode.J = number;
      return;
   }
   
   if( letter == 'K' )
   {
      if( gCode.G[1] == 2 || gCode.G[1] == 3 ) gCode.newAxisMove = true; // only flag new move if doing arc move
      gCode.K = number;
      return;
   }

   if( letter == 'A' )
   {
      gCode.A = number;
      return;
   }
   
   if( letter == 'B' )
   {
      gCode.B = number;
      return;
   }
   
   if( letter == 'C' )
   {
      gCode.C = number;
      return;
   }

   if( letter == 'U' )
   {
      gCode.U = number;
      return;
   }
   
   if( letter == 'V' )
   {
      gCode.V = number;
      return;
   }
   
   if( letter == 'W' )
   {
      gCode.W = number;
      return;
   }
  
   if( letter == 'D' )
   {
      gCode.D = number;
      return;
   }
   
   if( letter == 'H' )
   {
      gCode.H = number;
      return;
   }

   if( letter == 'L' )
   {
      gCode.L = number;
      return;
   }

   if( letter == 'N' )
   {
      gCode.N = number;
      return;
   }
   
   if( letter == 'Q' )
   {
      gCode.Q = number;
      return;
   }
   
   if( letter == 'R' )
   {
      gCode.R = number;
      return;
   }
  
   if( letter == 'S' )
   {
      gCode.S = number;
      return;
   }
   
   if( letter == 'T' )
   {
      gCode.T = number;
      return;
   }

}


bool readNextProgramLine()
{
   bool endOfBlockFound = false;
   char ch = getNextChar();

   gCode.startX = gCode.X; // save current location
   gCode.startY = gCode.Y;
   gCode.startZ = gCode.Z;

   while( ch != 10 && ch != 0 )  // iterate to end of the line
   {
      if( endOfBlockFound ) // ignore all characters after the EOB until CR
      {
         ch = getNextChar(); // throw away
      }
      else
      {
         if( ch == ';' || ch == '(' )
         {
            endOfBlockFound = true;
            ch = getNextChar();
         }
         else
         {
            if( ch > 96 && ch < 123 ) ch -= 32; // to upper

            if( ch > 64 && ch < 91 ) // Get Letter (ignore all else)
            {
               bool negative   = false;
               bool validNum   = false;
               int decimal     = 0;
               int32_t iNumber = 0;
               char letter     = ch;    // save letter for later
 
               ch = getNextChar();
 
               if( ch == '-' )  // check for negative (only valid if it is the first char after the letter)
               {
                  negative = true;
                  ch = getNextChar();
               }
               else if( ch == '+' )
               {
                  ch = getNextChar();  // ignore positive sign
               }

               while( ch - '0' <= 9 ) // get digits for int portion
               {
                  validNum = true; // at least one digit after the letter

                  iNumber = iNumber * 10 + (ch - '0'); // move previous value over one dec place and add new number

                  ch = getNextChar();
               }

               if( ch == '.' || ch == ',' )  // catch decimal point
               {
                  ch = getNextChar();
               }
 
               while( ch - '0' <= 9 && decimal < 5 ) // Get digits for decimal portion (up to 5)
               {
                  validNum = true; // at least one digit after the letter
                  
                  iNumber = iNumber * 10 + (ch - '0'); // move previous value over one dec place and add new number

                  decimal++; // increment number of decimal places
                  
                  ch = getNextChar();
               }
 
               if( validNum )
               {
                  float fNumber;
                  static const float decTable[6] = { 1.0f, 0.1f, 0.01f, 0.001f, 0.0001f, 0.00001f };
 
                  if( negative ) iNumber *= -1;
 
                  if( decimal )
                  {
                     fNumber = float( iNumber ) * decTable[decimal]; // decimal
                  }
                  else
                  {
                     fNumber = float( iNumber ); // integer
                  }
                  
                  setState( letter, fNumber );
               }
               else
               {
                  ch = getNextChar(); // if number is invalid, keep moving forward
               } 
            }
            else
            {
               ch = getNextChar(); // if letter is invalid, keep moving forward
            }
         }
      }
   } // end of main while loop

   if( ch == 0 ) return false; // return false if at end of file
   
   return true;
}


void executeCodeNow()
{
   //Group3();  // Abs/Inc selection

   //Group5();  // Feed Mode

   //Group6();  // Inch/Metric

   //Group7();  // Cutter comp

   //Group8();  // Tool length comp

   //Group10();  // Return plane select

   //Group11();  // Scaling

   //Group12();  // Work coordinate

   //Group15();  // Exact stop mode

   //Group16();  // Rotation

   movementOperations();  // group 1 -- placed last to allow modification by other G codes on the same line

}


void executeCodeDelayed()
{
   Group0();  // dwell, non-modal commands

   //Group2();  // Plane Selection

   //Group9();  // Canned cycles
    
   mCodes();  // miscellaneous, delayed execute

   KORE.delayedExecute = false;
}



