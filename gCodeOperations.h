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
         motion.addArc_Block(gCode.G[1], gCode.X, gCode.Y, gCode.F, arcCenterX, arcCenterY);
         break;
   }

   gCode.newAxisMove = false;
}


void addExtruderMove()
{
   gCode.newExtruderMove = false;
}


void movementOperations()
{
   if( gCode.newAxisMove && gCode.newExtruderMove ) // extrude while moving
   {
      addMovementBlock();
      addExtruderMove();
   }
   else if ( gCode.newAxisMove ) // move only
   {
      addMovementBlock();
      //Serial.println("!");
   }
   else if ( gCode.newExtruderMove ) // extrude only
   {
      addExtruderMove();
   }
}


void Group0()
{
   if( gCode.G[0] )
   {
      if( gCode.G[0] == 4 ) // dwell
      {
         motion.addDelay( max( gCode.P, 1 ) );
      }
      else if( gCode.G[0] == 5 ) // exact stop
      {
         motion.addDelay(10);
      }
      gCode.G[0] = 0;
   }
}


void Group2()
{
   if( gCode.G[2] )
   {

   }
}


void Group3()
{
   if( gCode.G[3] )
   {

   }
}


void Group5()
{
   if( gCode.G[5] )
   {

   }
}


void Group6()
{
   if( gCode.G[6] )
   {

   }
}


void Group7()
{
   if( gCode.G[7] )
   {

   }
}


void Group8()
{
   if( gCode.G[8] )
   {

   }
}


void Group9()
{
   if( gCode.G[9] )
   {

   }
}


void Group10()
{
   if( gCode.G[10] )
   {

   }
}


void Group11()
{
   if( gCode.G[11] )
   {

   }
}


void Group12()
{
   if( gCode.G[12] )
   {

   }
}


void Group15()
{
   if( gCode.G[15] )
   {

   }
}


void Group16()
{
   if( gCode.G[16] )
   {

   }
}


void mCodes()
{
   if( gCode.newMcode )
   {
      switch(gCode.M)
      {
         case 0:     // Stop
            break;
         
         case 1:     // Optional Stop
            break;
         
         case 2:     // Program End
            break;
         
         case 3:     // Spindle Start CW
            break;
         
         case 4:     // Spindle Start CCW
            break;

         case 5:     // Spindle Stop
            break;

         case 6:     // Tool Change
            break;
         
         case 7:     // Shower Coolant
            break;

         case 8:     // Coolant On
            break;

         case 9:     // Coolant Off
            break;

         case 30:    // Program end, reset
            break;

         case 109:    // Hot end on, DO wait
         case 104:    // Hot end on, NO wait
            extrude1HeaterTemp = int(gCode.S);
            gCode.S = 0.0f;
            break;

         default:
            break;

      }
      gCode.newMcode = false;
   }
}