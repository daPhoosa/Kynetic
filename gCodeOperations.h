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
         if( !gCode.lastMoveRapid )
         {
            gCode.lastMoveRapid = true;
            motion.addDwell_Block(10); // add delay when switching beteen rapids and feeds
            if( abs(gCode.startZ - gCode.Z) < 0.001 ) // no vertical component
            {
               float dx = gCode.startX - gCode.X;
               float dy = gCode.startY - gCode.Y;
               if( dx*dx + dy*dy > Z_HOP_MIN_DIST_SQ ) // first move is greater than min
               {
                  motion.addRapid_Block( gCode.startX, gCode.startY, gCode.startZ + AUTO_Z_HOP_HEIGHT ); // lift up
                  gCode.zHopActive = true;
               }
            }
         }
         
         if( gCode.zHopActive )
         {
            motion.addLinear_Block( gCode.X, gCode.Y, gCode.Z + AUTO_Z_HOP_HEIGHT, gCode.F);
         }
         else
         {
            //motion.addRapid_Block( gCode.X, gCode.Y, gCode.Z );
            motion.addLinear_Block( gCode.X, gCode.Y, gCode.Z, gCode.F);  // 3D printer gCode assumes that rapids obey feed rate...
         }
         
         break;

      case 1:
         if( gCode.lastMoveRapid )
         {
            gCode.lastMoveRapid = false;
            if( gCode.zHopActive )
            {
               gCode.zHopActive = false;
               motion.addRapid_Block( gCode.startX, gCode.startY, gCode.startZ ); // drop down
            }
            motion.addDwell_Block(10); // add delay when switching beteen rapids and feeds
         }
         motion.addLinear_Block( gCode.X, gCode.Y, gCode.Z, gCode.F);
         break;

      case 2:
      case 3:
         if( gCode.lastMoveRapid )
         {
            gCode.lastMoveRapid = false;
            if( gCode.zHopActive )
            {
               gCode.zHopActive = false;
               motion.addRapid_Block( gCode.startX, gCode.startY, gCode.startZ ); // drop down
            }
            motion.addDwell_Block(10); // add delay when switching beteen rapids and feeds
         }
         arcCenterX = gCode.startX + gCode.I;
         arcCenterY = gCode.startY + gCode.J;
         motion.addArc_Block(gCode.G[1], gCode.X, gCode.Y, gCode.F, arcCenterX, arcCenterY);
         break;

      default:
         break;
   }

   gCode.newAxisMove = false;
}


void movementOperations()
{
   if( gCode.newAxisMove && gCode.newExtruderMove ) // extrude while moving
   {
      addMovementBlock();
      motion.addExtrudeMM( gCode.E );
      gCode.newExtruderMove = false;
   }
   else if ( gCode.newAxisMove ) // move only
   {
      addMovementBlock();
   }
   else if ( gCode.newExtruderMove ) // extrude only
   {
      if( gCode.zHopActive )  // dummy block in current location to attach static extrude to
      {
         motion.addRapid_Block( gCode.X, gCode.Y, gCode.Z + AUTO_Z_HOP_HEIGHT );
      }
      else
      {
         motion.addRapid_Block( gCode.X, gCode.Y, gCode.Z ); 
      }
      motion.addExtrudeMM( gCode.E, gCode.F );
      gCode.newExtruderMove = false;
   }
}


void Group0()
{
   if( gCode.G[0] )
   {
      Vec3 motor;

      switch( gCode.G[0] )
      {
         case 4:
            motion.addDwell_Block( max( int(gCode.P), 1 ) );
            gCode.P = 0.0f;
            break;

         case 9:
            motion.addDwell_Block(10);
            break;

         case 28:
            KORE.runProgram = false;
            if( gCode.homeX || gCode.homeY || gCode.homeZ ) 
            {
               machine.startHome( gCode.homeX, gCode.homeY, gCode.homeZ );
               gCode.homeX = gCode.homeY = gCode.homeZ = false;
            }
            else
            {
               machine.startHome( true, true, true ); // home all axis if no axis is specified
            }
            break;

         case 29:
            break;

         case 92: // set position
            /*
            machine.invKinematics( gCode.X, gCode.Y, gCode.Z, motor.x, motor.y, motor.z ); // convert from cartesian to machine coordinates 
            A_motor.setPosition( motor.x );
            B_motor.setPosition( motor.y );
            C_motor.setPosition( motor.z );
            D_motor.setPosition( gCode.E );

            motion.setPosition( gCode.X, gCode.Y, gCode.Z, gCode.E );
            motion.startMoving();
            */

            Serial.print("X:"); Serial.print(gCode.X - gCode.startX);Serial.print(" Y:"); Serial.print(gCode.Y - gCode.startY);Serial.print(" Z:"); Serial.println(gCode.Z - gCode.startZ);

            gCode.workOffsetX += gCode.startX - gCode.X;
            gCode.workOffsetY += gCode.startY - gCode.Y;
            gCode.workOffsetZ += gCode.startZ - gCode.Z;

            D_motor.setPosition( gCode.E );
            motion.setPosE( gCode.E );
            motion.startMoving();

            //display("Set Position:  X:" + String(gCode.X, 2) + "  Y:" + String(gCode.Y, 2) + "  Z:" + String(gCode.Z, 2) + "  E:" + String(gCode.E, 2) + '\n');
            break;
         
         default:
            break;
      }
   
      gCode.G[0] = 0;
   }
}

/*
void Group2()
{
   if( gCode.G[2] )
   {

   }
}
*/
/*
void Group3()
{
   if( gCode.G[3] )
   {

   }
}
*/
/*
void Group5()
{
   if( gCode.G[5] )
   {

   }
}
*/
/*
void Group6()
{
   if( gCode.G[6] )
   {

   }
}
*/
/*
void Group7()
{
   if( gCode.G[7] )
   {

   }
}
*/
/*
void Group8()
{
   if( gCode.G[8] )
   {

   }
}
*/
/*
void Group9()
{
   if( gCode.G[9] )
   {

   }
}
*/
/*
void Group10()
{
   if( gCode.G[10] )
   {

   }
}
*/
/*
void Group11()
{
   if( gCode.G[11] )
   {

   }
}
*/
/*
void Group12()
{
   if( gCode.G[12] )
   {

   }
}
*/
/*
void Group15()
{
   if( gCode.G[15] )
   {

   }
}
*/
/*
void Group16()
{
   if( gCode.G[16] )
   {

   }
}
*/

void mCodes()
{
   if( gCode.newMcode )
   {
      switch(gCode.M)
      {
         case 0:     // Stop
            KORE.manualPauseActive = true;
            break;
         
         case 1:     // Optional Stop
            KORE.manualPauseActive = true;
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

         case 82:    // Extrude Absolute Mode
            gCode.extrudeAbsoluteMode = true;
            break;

         case 83:    // Extrude Relative Mode
            gCode.extrudeAbsoluteMode = false;
            break;

         case 109:    // Hot end on, DO wait
            extStartTime = millis();
            KORE.extrude1_wait = true;
            KORE.extrude1TargetTemp = min( int(gCode.S + 0.5f), MAX_EXTRUDER1_TEMP );
            gCode.S = 0.0f;
            break;

         case 104:    // Hot end on, NO wait
            KORE.extrude1_wait = false;
            KORE.extrude1TargetTemp = min( int(gCode.S + 0.5f), MAX_EXTRUDER1_TEMP );
            gCode.S = 0.0f;
            break;

         case 190:    // Bed on, DO wait
            bedStartTime = millis();
            KORE.bed_wait = true;
            KORE.bedTargetTemp = min( int(gCode.S + 0.5f), MAX_BED_TEMP );
            gCode.S = 0.0f;
            break;

         case 140:    // Bed on, NO wait
            KORE.bed_wait = false;
            KORE.bedTargetTemp = min( int(gCode.S + 0.5f), MAX_BED_TEMP );
            gCode.S = 0.0f;
            break;

         default:
            break;

      }
      gCode.newMcode = false;
   }
}