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
      if( !endOfBlockFound ) // ignore all characters after the EOB until CR
      {
         if( ch == ';' )
         {
            endOfBlockFound = true;
         }
         else
         {
            if( ch > 64 && ch < 91 ) // Get Letter (ignore all else)
            {
               bool negative = false;
               float decimal = 1.0f;
               float number  = 0.0f;
               char letter   = ch;

               ch = getNextChar();

               if(ch = '-')  // check for negative (only valid if it is the first char after the letter)
               {
                  negative = true;
                  ch = getNextChar();
               }

               while( ch > 47 && ch < 58 || ch == 46 ) // Get Number (or decimal point )
               {
                  if( ch == 46 )
                  {
                     decimal = 0.1f;
                  }
                  else
                  {
                     if( decimal > 0.9f )
                     {
                        number = number * 10.0f + float( ch - 48 );  // whole numbers
                     }
                     else
                     {
                        number += float( ch - 48 ) * decimal; // decimal part
                        decimal *= 0.1f;
                     }
                  }

                  ch = getNextChar();
               }
               if( negative ) number *= -1.0f;
               // assign number to letter address
            }
         }
      
      
      

   }
   
   
}


bool streamToBlock(const char & ch)  // returns true when a block is complete
{
   static bool blockComplete = false;
   static bool lineComplete = false;
   static char field;
   
   if(ch == '\r')  // end of line
   {
      field = 0;
      lineComplete = true;
      return false;
   }
   
   if(field == ';')
   {
      return false; // ignore characters after the EOB
   }
   
   switch(ch)
   {
      case ';' :      // everything after this point is ignored
         field = ';';
         lineComplete = true;
         break;

      case 'G' :    
      case 'M' :    
      case 'X' :    
      case 'Y' :    
      case 'Z' :    
      case 'E' : 
      case 'F' : 
      case 'S' : 
         field = ch;
         return false;        
      
      default :
         break; 
   }
   
   return false;
}
