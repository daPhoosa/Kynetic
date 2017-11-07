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

   bool newMove;
   
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


void readNextLine()
{
   bool endOfBlockFound = false;
   char ch = getNextChar();
   int32_t number;
   
   while( ch != '\r' && ch != 0 )  // iterate to end of the line
   {
      if( endOfBlockFound ) // ignore all characters after the EOB until CR
      {
         ch = getNextChar(); // throw away
      }
      else
      {
         if( ch == ';' )
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
}


void setState( char letter, float number )
{
   gCode.newMove = false;

   // list most common letters first to avoid uneeded compares

   if( letter == 'X' )
   {
      float diff = abs( gCode.X - number );
      if( diff > 0.0009f )
      {
         gCode.newMove = true;
         gCode.X = number;
      }
      return;
   }

   if( letter == 'Y' )
   {
      float diff = abs( gCode.Y - number );
      if( diff > 0.0009f )
      {
         gCode.newMove = true;
         gCode.Y = number;
      }
      return;
   }

   if( letter == 'E' )
   {
      float diff = abs( gCode.E - number );
      if( diff > 0.0009f )
      {
         gCode.newMove = true;
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
         gCode.newMove = true;
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
<<<<<<< HEAD

   if( letter == 'M' )
   {
      int num = int(number);

=======

   if( letter == 'M' )
   {
      int num = int(number);

>>>>>>> de66c7874c72af2fd892ad60c3ad157eb8d5a71b
      gCode.M = num;
      return;
   }

}
