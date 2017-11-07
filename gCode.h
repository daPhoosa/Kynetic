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


struct gCode_state_machine_t
{
   byte G[17]; // G code groups are 0 - 16
   int M;
   
   float A, B, C, D, E, F, H, I, J, K, L, N, P, Q, R, S, T, U, V, W, X, Y, Z; // G, M, O intentionally omitted 

   float startX, startY, startZ;

   bool newAxisMove, newExtruderMove;
   
} gCode;



char getNextChar()
{
   if( file.available() )
   {
      char ch;
      file.read(&ch, 1);
      return ch;
   }
   return 0;
}


bool readNextProgramLine()
{
   bool endOfBlockFound = false;
   char ch = getNextChar();
   int32_t number;

   gCode.startX = gCode.X; // save current location
   gCode.startY = gCode.Y;
   gCode.startZ = gCode.Z;

   while( ch != '\r' && ch != 0 )  // iterate to end of the line
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
            if( ch > 64 && ch < 91 ) // Get Letter (ignore all else)
            {
               bool negative = false;
               float decimal = 1.0f; // arbitrary number above 0.1
               float number  = 0.0f;
               char letter   = ch;

               ch = getNextChar();

               if(ch == '-')  // check for negative (only valid if it is the first char after the letter)
               {
                  negative = true;
                  ch = getNextChar();
               }

               while( (ch > 47 && ch < 58) || ch == '.' || ch == ',' ) // Get Number (or decimal point )
               {
                  if( ch == '.' || ch == ',' )
                  {
                     if( decimal > 0.101f ) 
                     {
                        decimal = 0.1f; // only change if a decimal point has not been seen previously
                     }
                     else
                     {
                        // ( in the future multiple decimal points should probably generate an alarm, for now just ignore )
                     }
                  }
                  else
                  {
                     if( decimal > 0.101f )
                     {
                        number = number * 10.0f + float( ch - 48 );  // int component
                     }
                     else
                     {
                        number += float( ch - 48 ) * decimal; // decimal component
                        decimal *= 0.1f;
                     }
                  }

                  ch = getNextChar();
               }
               if( negative ) number *= -1.0f;
               setState( letter, number );
            }
         }
      }
   }

   if( ch == 0) return false; // return false if at end of file
   
   return true;
}


void setState( char letter, float number )
{

   // list most common letters first to avoid uneeded compares

   if( letter == 'X' )
   {
      float diff = abs( gCode.X - number );
      if( diff > 0.0009f )
      {
         gCode.newAxisMove = true;
         gCode.X = number;
      }
      return;
   }

   if( letter == 'Y' )
   {
      float diff = abs( gCode.Y - number );
      if( diff > 0.0009f )
      {
         gCode.newAxisMove = true;
         gCode.Y = number;
      }
      return;
   }

   if( letter == 'E' )
   {
      float diff = abs( gCode.E - number );
      if( diff > 0.0009f )
      {
         gCode.newExtruderMove = true;
         gCode.E = number;
      }
      return;
   }

   if( letter == 'F' )
   {
      gCode.F = number;
      return;
   }

   if( letter == 'Z' )
   {
      float diff = abs( gCode.Z - number );
      if( diff > 0.0009f )
      {
         gCode.newAxisMove = true;
         gCode.Z = number;
      }
      return;
   }

   if( letter == 'G' )
   {
      byte num = byte(number);

      switch( num )
      {
         case 4 :    // Group 0
         case 9 :
         case 28:
         case 29:
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
      }
      return;
   }

   if( letter == 'M' )
   {
      int num = int(number);

      gCode.M = num;
      return;
   }

   if( letter == 'P' )
   {
      gCode.P = num;
      return;
   }
   
   if( letter == 'I' )
   {
      gCode.I = num;
      return;
   }
   
   if( letter == 'J' )
   {
      gCode.J = num;
      return;
   }
   
   if( letter == 'K' )
   {
      gCode.K = num;
      return;
   }

   if( letter == 'A' )
   {
      gCode.A = num;
      return;
   }
   
   if( letter == 'B' )
   {
      gCode.B = num;
      return;
   }
   
   if( letter == 'C' )
   {
      gCode.C = num;
      return;
   }

   if( letter == 'U' )
   {
      gCode.U = num;
      return;
   }
   
   if( letter == 'V' )
   {
      gCode.V = num;
      return;
   }
   
   if( letter == 'W' )
   {
      gCode.W = num;
      return;
   }
  
   if( letter == 'D' )
   {
      gCode.D = num;
      return;
   }
   
   if( letter == 'H' )
   {
      gCode.H = num;
      return;
   }

   if( letter == 'L' )
   {
      gCode.L = num;
      return;
   }

   if( letter == 'N' )
   {
      gCode.N = num;
      return;
   }
   
   if( letter == 'Q' )
   {
      gCode.Q = num;
      return;
   }
   
   if( letter == 'R' )
   {
      gCode.R = num;
      return;
   }
  
   if( letter == 'S' )
   {
      gCode.S = num;
      return;
   }
   
   if( letter == 'T' )
   {
      gCode.T = num;
      return;
   }

}


void executeCode()
{

   if( gCode.newAxisMove && gCode.newExtruderMove ) // extrude while moving
   {
      addMovementBlock();
      //motion.addExtrude();
   }
   else if ( gCode.newAxisMove ) // move only
   {
      addMovementBlock();
   }
   else if ( gCode.newExtruderMove ) // extrude only
   {
      // motion.addExtrudeOnly();
   }

   gCode.newAxisMove = false;
   gCode.newExtruderMove = false;

   // Non movement commands      
   

}


void addMovementBlock()
{
     float arcCenterX, arcCenterY;

      switch (gCode.G[1])
      {
         case 0:
            motion.addLinear_Block(0, gCode.X, gCode.Y, gCode.Z, MAX_VELOCITY);
            break;

         case 1:
            motion.addLinear_Block(1, gCode.X, gCode.Y, gCode.Z, gCode.F);
            break;

         case 2:
         case 3:
            arcCenterX = gCode.startX + gCode.I;
            arcCenterY = gCode.startY + gCode.J;
            motion.addArc_Block(gCode.G[1], gCode.X, gCode.Y, gCode.Z, gCode.F, arcCenterX, arcCenterY);
            break;

      }   
}
