/*
   SmoothMove
   Phillip Schmidt
   v0.1

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

#include "SmoothMove.h"


void SmoothMove::startMoving() //
{
   // SetPosition(...) must be used before this is run

   blockCount    = 0; // "forget" all previous blocks
   lookAheadTime = 0;
   segmentIndex  = 3;
   segmentTime   = 0;

   addRapid_Block( X_end, Y_end, Z_end ); // add dummy block at current position, zero length
   addDelay( 100 );   // give time for more blocks to be added to buffer before starting to move

   currentBlockIndex = newBlockIndex; // execute the block that was just added

   moveBuffer[currentBlockIndex].accelTime = 0; // manually set these to zero since trajectory planning doesn't touch the curent block
   moveBuffer[currentBlockIndex].velTime   = 0;
   moveBuffer[currentBlockIndex].decelTime = 0;

   motionStopped    = false;
   segmentStartTime = micros();
}


void SmoothMove::abortMotion() // all blocks in queue will be lost on restart
{
   motionStopped = true;
}


void SmoothMove::advancePostion() // this moves forward along the acc/dec trajectory
{
   if( blockCount == 0 || motionStopped )
   {
      // no blocks ready to be executed
      velocityNow = 0.0f;
      segmentStartTime = micros();
   }
   else
   {
      uint32_t deltaTime = micros() - segmentStartTime;

      //  check if the next segment has been entered  -- while loop is used to cross multiple zero length segments
      while( deltaTime > segmentTime )
      {
         segmentStartTime += segmentTime; // advance start time by previous segment time
         deltaTime -= segmentTime;

         switch( segmentIndex )
         {
            case 4 : // switch to ACCELERATION
               removeOldBlock(); // previous block complete, index to next block

               segmentTime    = moveBuffer[currentBlockIndex].accelTime;
               lookAheadTime -= moveBuffer[currentBlockIndex].accelTime; // segment time is removed as soon as the segment is started
               segmentIndex = 0;
               break;

            case 0 : // switch to CONST VELOCITY
               segmentTime    = moveBuffer[currentBlockIndex].velTime;
               lookAheadTime -= moveBuffer[currentBlockIndex].velTime;
               segmentIndex = 1;
               break;

            case 1 : // switch to DECELERATION
               segmentTime    = moveBuffer[currentBlockIndex].decelTime;
               lookAheadTime -= moveBuffer[currentBlockIndex].decelTime;
               segmentIndex = 2; // move to next block
               break;

            case 2 : // switch to DWELL
               segmentTime  = moveBuffer[currentBlockIndex].dwell;
               segmentIndex = 3;
               break;

            case 3 : // switch to NEXT block
               if( blockCount > 1 &&                                       // another block must exist
                   !moveBuffer[currentBlockIndex].staticExtrude &&         // wait until static extrude is complete
                   moveBuffer[nextBlockIndex(currentBlockIndex)].ready )   // wait until next block is ready
               {
                  segmentIndex = 4;
                  segmentTime  = 0;
               }
               else  // WAIT for next block
               {
                  moveBuffer[currentBlockIndex].dwell = 1; // mark current block as having had a stop
                  segmentTime = 999UL; // force 1ms of dwell before checking again ( 999 is a special number used in blockQueueComplete() )
               }
               break;
         }
      }

      float t = float(deltaTime) * (1.0f / 1000000.0f); // time in seconds
      float t_2, t_3, t_4, t_5;

      switch(segmentIndex)  // compute current position in the block
      {
         case 0 : // state: Accel
            t_2 = t * t;
            t_3 = t * t_2;
            t_4 = t * t_3;
            t_5 = t * t_4;
            blockPosition = moveBuffer[currentBlockIndex].Acc.P_5 * t_5 +
                            moveBuffer[currentBlockIndex].Acc.P_4 * t_4 +
                            moveBuffer[currentBlockIndex].Acc.P_3 * t_3 +
                            moveBuffer[currentBlockIndex].Acc.C_1 * t;
            velocityNow   = moveBuffer[currentBlockIndex].Acc.V_4 * t_4 +
                            moveBuffer[currentBlockIndex].Acc.V_3 * t_3 +
                            moveBuffer[currentBlockIndex].Acc.V_2 * t_2 +
                            moveBuffer[currentBlockIndex].Acc.C_1;
            break;

         case 1 : // state: Const Vel
            blockPosition = moveBuffer[currentBlockIndex].targetVel * t + moveBuffer[currentBlockIndex].accelEndPoint;
            velocityNow = moveBuffer[currentBlockIndex].targetVel;
            break;

         case 2 : // state: Decel
            t_2 = t * t;
            t_3 = t * t_2;
            t_4 = t * t_3;
            t_5 = t * t_4;
            blockPosition = moveBuffer[currentBlockIndex].Dec.P_5 * t_5 +
                            moveBuffer[currentBlockIndex].Dec.P_4 * t_4 +
                            moveBuffer[currentBlockIndex].Dec.P_3 * t_3 +
                            moveBuffer[currentBlockIndex].Dec.C_1 * t   +
                            moveBuffer[currentBlockIndex].velEndPoint;
            velocityNow   = moveBuffer[currentBlockIndex].Dec.V_4 * t_4 +
                            moveBuffer[currentBlockIndex].Dec.V_3 * t_3 +
                            moveBuffer[currentBlockIndex].Dec.V_2 * t_2 +
                            moveBuffer[currentBlockIndex].Dec.C_1;
            break;

         case 3 : // state: Dwell
         case 4 : // state: Wait for next block
            blockPosition = moveBuffer[currentBlockIndex].length; // stop at end of current block
            velocityNow = 0.0f;
            break;
      }
   }
}


void SmoothMove::setMaxStartVel(const int & index)  // Junction Velocity
{
   int prevBlock = previousBlockIndex(index);

   if( blockCount > 1 && !moveBuffer[prevBlock].dwell )
   {
      float prevBlockDist = moveBuffer[prevBlock].length - junctionRadius;

      int bCount = blockCount;

      while( prevBlockDist < 0.0f && bCount > 2 )  // look backwards past very short blocks
      {
         bCount--;
         prevBlock = previousBlockIndex(prevBlock);
         prevBlockDist = moveBuffer[prevBlock].length + prevBlockDist;
      }

      float x1, y1, z1;
      float x2, y2, z2;
      getPos( x1, y1, z1, index, junctionRadius );
      getPos( x2, y2, z2, prevBlock, prevBlockDist );
      float maxAccel = min( moveBuffer[index].maxAccel, moveBuffer[prevBlock].maxAccel ); // use lower acceleration rate

      x1 -= x2; // difference in positions
      y1 -= y2;
      z1 -= z2;
      float pointDistSq = x1 * x1 + y1 * y1 + z1 * z1;

      float radius = sqrtf( pointDistSq * junctionRadiusSq / ( 4.00001f * junctionRadiusSq - pointDistSq ));

      float junctionVelSq = maxAccel * radius;

      float minBlockVel = min( moveBuffer[index].targetVel, moveBuffer[prevBlock].targetVel );

      if( junctionVelSq < minBlockVel * minBlockVel )
      {
         moveBuffer[index].maxStartVel = sqrtf(junctionVelSq);
         moveBuffer[index].fastJunction = false;
      }
      else
      {
         moveBuffer[index].maxStartVel = minBlockVel;
         moveBuffer[index].fastJunction = true;
      }
   }
   else
   {
      moveBuffer[index].maxStartVel = 0.0f;  // first block always starts at zero vel and blocks after an exact stop
      moveBuffer[index].fastJunction = true; // no point in smoothing if coming from a dead stop
   }
}


void SmoothMove::minJerkTrajectory()
{
   /*
      Block Diagram:
      [current block][  ][  ][  ][  ][  ][newest block]
      ---direction of execution--->
      [current block][  ][  ][  ][  ][  ][start][exit]

      STRATEGY:
         - Use constant acceleration to compute time intervals and velocity changes
            - Traverse block queue backwards and reduce velocities at block borders to insure adequate acc/dec time 
            - Traverse block queue forward and compute acc/vel/dec times and distances
         - Compute minimum jerk s-curve transitions to fit the acceleration and deceleration intervals
   */

   int exit  = newBlockIndex;
   int start = previousBlockIndex(exit);

   moveBuffer[exit].ready = false;

   xVel[exit]    = 0.0f;   // newest block always ends at zero
   xVel_Sq[exit] = 0.0f;

   for( int i = blockCount - 2; i > 0 ; i-- )
   {
      // Iterate through the active blocks backwards (newest to oldest)
      //    On the first pass, only border velocities are changed
      //    These can only be made slower, never faster
      //    Reminder: the current (oldest) block should not be adjusted

      moveBuffer[start].ready = false;

      xVel[start] = moveBuffer[exit].maxStartVel;
      xVel_Sq[start] = xVel[start] * xVel[start];

      float distToDeltaVel = (xVel_Sq[start] - xVel_Sq[exit]) * moveBuffer[exit].accelInverseHalf;

      if(distToDeltaVel > moveBuffer[exit].length) // not enough room to decel from startVel to endVel
      {
         xVel_Sq[start] = xVel_Sq[exit] + moveBuffer[exit].accelDouble * moveBuffer[exit].length; // set startVel lower
         xVel[start]    = sqrtf(xVel_Sq[start]);
      }
      else if(distToDeltaVel < -moveBuffer[exit].length) // not enough room to accel from startVel to endVel
      {
         xVel_Sq[exit] = xVel_Sq[start] + moveBuffer[exit].accelDouble * moveBuffer[exit].length; // set exitVel lower
         xVel[exit]    = sqrtf(xVel_Sq[exit]);
      }

      // move backwards through block queue
      exit  = previousBlockIndex(exit);
      start = previousBlockIndex(start);
   }

   lookAheadTime = 0; // zero before re-summing total
   if( segmentIndex < 1 ) lookAheadTime += moveBuffer[currentBlockIndex].velTime;    // include const vel time
   if( segmentIndex < 2 ) lookAheadTime += moveBuffer[currentBlockIndex].decelTime;  // include decel time

   for( int i = blockCount - 1; i > 0 ; i-- )
   {
      // Iterate forward
      //    check and set boundary velocities
      //    set position and time variables

      float distToDeltaVel = (xVel_Sq[exit] - xVel_Sq[start]) * moveBuffer[exit].accelInverseHalf;

      if(distToDeltaVel > moveBuffer[exit].length) // not enough room to accel from startVel to endVel
      {
         xVel_Sq[exit] = xVel_Sq[start] + moveBuffer[exit].accelDouble * moveBuffer[exit].length; // set exitVel lower
         xVel[exit]    = sqrtf(xVel_Sq[exit]);

         moveBuffer[exit].peakVel = xVel[start];  // shouldn't need this...

         moveBuffer[exit].accelTime     = uint32_t(( xVel[exit] - xVel[start] ) * moveBuffer[exit].accelInverse * 1000000.0f);
         moveBuffer[exit].accelEndPoint = moveBuffer[exit].length;

         moveBuffer[exit].velTime       = 0;
         moveBuffer[exit].velEndPoint   = moveBuffer[exit].accelEndPoint;

         moveBuffer[exit].decelTime     = 0;
         moveBuffer[exit].decelLength   = 0.0f;
      }
      else  // Compute accel and decel
      {
         moveBuffer[exit].decelLength   = ( moveBuffer[exit].targetVel_Sq - xVel_Sq[exit]  ) * moveBuffer[exit].accelInverseHalf;

         moveBuffer[exit].accelEndPoint = ( moveBuffer[exit].targetVel_Sq - xVel_Sq[start] ) * moveBuffer[exit].accelInverseHalf;

         float constVelLength = moveBuffer[exit].length - moveBuffer[exit].decelLength - moveBuffer[exit].accelEndPoint;

         // Check for enough room to execute both
         if(constVelLength > 0.0f)  // accel should end before const vel
         {
            // enough room for both accel to and decel from targetVel

            moveBuffer[exit].peakVel = moveBuffer[exit].targetVel;

            moveBuffer[exit].velEndPoint = constVelLength + moveBuffer[exit].accelEndPoint;

            moveBuffer[exit].accelTime = uint32_t(( moveBuffer[exit].targetVel - xVel[start] ) * moveBuffer[exit].accelInverse * 1000000.0f);
            moveBuffer[exit].velTime   = uint32_t(( moveBuffer[exit].velEndPoint - moveBuffer[exit].accelEndPoint) / moveBuffer[exit].targetVel * 1000000.0f);
            moveBuffer[exit].decelTime = uint32_t(( moveBuffer[exit].targetVel - xVel[exit]  ) * moveBuffer[exit].accelInverse * 1000000.0f);
         }
         else  // peaked acceleration, targetVel not reached
         {
            float halfExcessLength = constVelLength * 0.5f;  // negative

            moveBuffer[exit].accelEndPoint += halfExcessLength;
            moveBuffer[exit].velEndPoint    = moveBuffer[exit].accelEndPoint; // zero length
            moveBuffer[exit].decelLength   += halfExcessLength;

            moveBuffer[exit].peakVel   = sqrtf( xVel_Sq[start] + moveBuffer[exit].accelDouble * moveBuffer[exit].accelEndPoint );

            moveBuffer[exit].accelTime = uint32_t(( moveBuffer[exit].peakVel - xVel[start]) * moveBuffer[exit].accelInverse * 1000000.0f);
            moveBuffer[exit].velTime   = 0;
            moveBuffer[exit].decelTime = uint32_t(( moveBuffer[exit].peakVel - xVel[exit] ) * moveBuffer[exit].accelInverse * 1000000.0f);
         }
      }

      lookAheadTime += moveBuffer[exit].accelTime + moveBuffer[exit].velTime + moveBuffer[exit].decelTime;

      if( moveBuffer[exit].accelTime > 0 ) // generate min jerk coefficients for acceleration
      {
         getTransCoef( moveBuffer[exit].accelTime, moveBuffer[exit].accelEndPoint, xVel[start], moveBuffer[exit].peakVel, moveBuffer[exit].Acc );
      }

      if( moveBuffer[exit].decelTime > 0 ) // generate min jerk coefficients for deceleration
      {
         getTransCoef( moveBuffer[exit].decelTime, moveBuffer[exit].decelLength, moveBuffer[exit].peakVel, xVel[exit], moveBuffer[exit].Dec );
      }

      moveBuffer[exit].ready = true;

      // move forward in block queue
      exit  = nextBlockIndex(exit);
      start = nextBlockIndex(start);
   }
   moveBuffer[newBlockIndex].ready = true;
}


void SmoothMove::getTargetLocation(float & x, float & y, float & z) // call to get current cartesian position
{

   if(blockCount == 0) // if no blocks are queued up, return current end point
   {
      x = X_end;
      y = Y_end;
      z = Z_end;
      return;
   }

   // SYMETRIC SMOOTHING
   if( pathSmoothingOff )          // smoothing turned off
   {
      getPos( x, y, z, currentBlockIndex, blockPosition ); // get current position
      return;
   }

   float smoothingPosStart = blockPosition;
   float smoothingPosEnd   = blockPosition;

   int smoothingIndexStart = currentBlockIndex;
   int smoothingIndexEnd   = currentBlockIndex;

   float d1 = bwdPoint( smoothingPosStart, smoothingIndexStart, cornerRoundDist );
   float d2 = fwdPoint( smoothingPosEnd, smoothingIndexEnd, d1 );
   if( d2 < d1 )
   {
      smoothingPosStart   = blockPosition;
      smoothingIndexStart = currentBlockIndex;
      d1 = bwdPoint( smoothingPosStart, smoothingIndexStart, d2 );
   }

   /*
   Serial.print(millis()); Serial.print("\t");
   Serial.print(moveBuffer[currentBlockIndex].dwell); Serial.print("\t");
   Serial.print(d1, 3); Serial.print("\t");
   Serial.print(blockPosition,     3); Serial.print("\t");
   Serial.println(d2, 3);
   

   /*
   float smoothingRadius = min( cornerRoundDist, velocityNow * velocityNow * moveBuffer[currentBlockIndex].accelInverseHalf - 0.001f );

   if( pathSmoothingOff ||          // smoothing turned off
       smoothingRadius < 0.0001f )   // return current position without smoothing if velocity is very low
   {
      getPos( x, y, z, currentBlockIndex, blockPosition ); // get current position
      return;
   }

   float smoothingPosStart = blockPosition - smoothingRadius;
   float smoothingPosEnd   = blockPosition + smoothingRadius;

   int smoothingIndexStart = currentBlockIndex;
   int smoothingIndexEnd   = currentBlockIndex;

   
   
   while( smoothingPosStart < 0.0f )   // find start point in previous blocks
   {
      int i = previousBlockIndex(smoothingIndexStart);
      if( !moveBuffer[i].dwell ) // don't smooth across a dwell
      {
         smoothingIndexStart = i;
         smoothingPosStart  += moveBuffer[smoothingIndexStart].length;
      }
      else
      {
         smoothingPosStart = 0.0f; // stop at beginning of block
      }
   }

   int bCnt = blockCount - 1;

   while( smoothingPosEnd > moveBuffer[smoothingIndexEnd].length && bCnt > 0 ) // find end point in future blocks
   {
      bCnt--;

      if( !moveBuffer[smoothingIndexEnd].dwell )
      {
         smoothingPosEnd  -= moveBuffer[smoothingIndexEnd].length;
         smoothingIndexEnd = nextBlockIndex(smoothingIndexEnd);
      }
      else
      {
         smoothingPosEnd = moveBuffer[smoothingIndexEnd].length; // stop at end of block
      }
   }
   */

   if( smoothingIndexStart == smoothingIndexEnd ||                  // do not smooth if both points lie in the current block
       moveBuffer[nextBlockIndex(currentBlockIndex)].fastJunction ) // do not smooth if next junction does not force deceleration
   {
      getPos( x, y, z, currentBlockIndex, blockPosition ); // get current position
      return;
   }

   float x2, y2, z2;

   getPos( x,  y,  z,  smoothingIndexStart, smoothingPosStart); // smoothing start position
   getPos( x2, y2, z2, smoothingIndexEnd,   smoothingPosEnd);   // smoothing end position is in this block

   x = ( x + x2 ) * 0.5f;  // average two smoothing points
   y = ( y + y2 ) * 0.5f;
   z = ( z + z2 ) * 0.5f;
}


float SmoothMove::fwdPoint( float & pos, int & index, const float dist )   // forward
{
   int bCount = blockCount;
   pos += dist;

   while( pos > moveBuffer[index].length ) // extends past end of block
   {
      if( moveBuffer[index].dwell || bCount == 1 ) // dont go past end of block if a dwell, or last block
      {
         float d = dist - pos + moveBuffer[index].length;   // distance minus truncated amount
         pos = moveBuffer[index].length;                 // set position to end of block
         return d;
      }
      else
      {
         bCount--;
         pos -= moveBuffer[index].length;    // position in next block
         index = nextBlockIndex(index);
      }
   }
   return dist;   // return full distance
}


float SmoothMove::bwdPoint( float & pos, int & index, const float dist )   // backwards
{
   float length = 0.0f;
   pos -= dist;

   while( pos < 0.0f ) // extends before start of block
   {
      int i = previousBlockIndex(index);
      length += moveBuffer[i].length;

      if( moveBuffer[i].dwell ) // dont go past start of block if prev has a dwell
      {
         float d = length - pos;   // actual distance
         pos = 0.0f;               // set position to start of block
         return d;
      }
      else
      {  
         index = i;
         pos += moveBuffer[index].length;    // position in next block
      }
   }
   return dist;   // return full distance
}


void SmoothMove::getPos(float & x, float & y, float & z, int index, float position)  // maps from accel trajectory to 3D space
{
   float angle;

   switch(moveBuffer[index].moveType)
   {
      case Linear :
         x = moveBuffer[index].X_vector * position + moveBuffer[index].X_start;
         y = moveBuffer[index].Y_vector * position + moveBuffer[index].Y_start;
         z = moveBuffer[index].Z_vector * position + moveBuffer[index].Z_start;
         break;

      case ArcCW  :
      case ArcCCW :
         if(moveBuffer[index].moveType == ArcCW)
         {
            angle = -position / moveBuffer[index].radius;
         }
         else
         {
            angle = position / moveBuffer[index].radius;
         }
         angle += moveBuffer[index].startAngle;
         x = moveBuffer[index].X_vector + moveBuffer[index].radius * cosf(angle);
         y = moveBuffer[index].Y_vector + moveBuffer[index].radius * sinf(angle);
         z = moveBuffer[index].Z_vector * position + moveBuffer[index].Z_start;
         break;
   }
}


float SmoothMove::getExtrudeLocationMM()
{
   static uint32_t lastTime;

   uint32_t timeNow = micros();
   float  deltaTime = float(timeNow - lastTime) * (1.0f / 1000000.0f);
   lastTime = timeNow;

   if( moveBuffer[currentBlockIndex].staticExtrude ) // extrude with no head movement
   {
      static float extrudePos = 0.0f;
      static float velocity = 0.0f;
      bool endFound = false;

      if( moveBuffer[currentBlockIndex].extrudeDist > 0.0f )
      {
         float decelVel = min( sqrtf( 2.0f * (moveBuffer[currentBlockIndex].extrudeDist - extrudePos) * extrudeAccel ), extrudeVel );
         velocity = min( velocity + extrudeAccel * deltaTime, decelVel );
         //Serial.println(velocity);
         extrudePos += velocity * deltaTime;
         if( extrudePos > moveBuffer[currentBlockIndex].extrudeDist ) endFound = true;
      }
      else  // negative extrude
      {
         float decelVel = min( sqrtf( 2.0f * (extrudePos - moveBuffer[currentBlockIndex].extrudeDist) * extrudeAccel ), extrudeVel );
         //Serial.println(decelVel);
         velocity = min( velocity + extrudeAccel * deltaTime, decelVel );
         //Serial.println(velocity);
         extrudePos -= velocity * deltaTime;
         if( extrudePos < moveBuffer[currentBlockIndex].extrudeDist ) endFound = true;
      }

      //Serial.println(extrudePos);

      if( endFound ) // end of extrude reached
      {
         extrudePos = 0.0f;
         velocity = 0.0f;
         moveBuffer[currentBlockIndex].staticExtrude = false;

         extrudeMachPos += moveBuffer[currentBlockIndex].extrudeDist; // force final position to be exact
         return extrudeMachPos;
      }
      return extrudeMachPos + extrudePos; // middle of extrude
   }
   else  // extrude while moving
   {
      float e = moveBuffer[currentBlockIndex].extrudeScaleFactor * blockPosition + extrudeMachPos;
      /*
      if( moveBuffer[currentBlockIndex].extrudeScaleFactor > 0.00001f ) // apply velocity compensation when extruding forward
      {
         e += extrudeVelocityAdvance * velocityNow;
      }
      */
      return e;
   }
}


void SmoothMove::getTransCoef( const uint32_t & time, const float & pos_end, const float & vel_start, const float & vel_end, min_jerk_coeffients_t & X )
{

   float t = float(time) * (1.0f / 1000000.0f); // convert from us to s

   float t_2 = t * t;
   float t_3 = t * t_2;
   float t_4 = t * t_3;
   float t_5 = t * t_4;

   /*
   // Agugmented matrix
   float M[6][7] = {{20.0f*t_3, 12.0f*t_2,  6.0f*t,   2.0f, 0.0f, 0.0f, 0.0f},// C_5 - acceleration end
                    { 5.0f*t_4,  4.0f*t_3, 3.0*t_2, 2.0f*t, 1.0f, 0.0f, vel_end},   // C_4 - vel end
                    {      t_5,       t_4,     t_3,    t_2,    t, 1.0f, pos_end},   // C_3 - pos end
                    {     0.0f,      0.0f,    0.0f,   2.0f, 0.0f, 0.0f, 0.0f},      // C_2 - acceleration start
                    {     0.0f,      0.0f,    0.0f,   0.0f, 1.0f, 0.0f, vel_start}, // C_1 - vel start
                    {     0.0f,      0.0f,    0.0f,   0.0f, 0.0f, 1.0f, 0.0f}};     // C_0 - pos start
   */
   // eliminate trivial colums and rows
   float M[4][5] = {{ 20.0f*t_3, 12.0f*t_2,  6.0f*t, 0.0f, 0.0f      },  // C_5 - acceleration end
                    {  5.0f*t_4,  4.0f*t_3, 3.0*t_2, 1.0f, vel_end   },  // C_4 - vel end
                    {       t_5,       t_4,     t_3,    t, pos_end   },  // C_3 - pos end
                    {      0.0f,      0.0f,    0.0f, 1.0f, vel_start }}; // C_1 - vel start

   // *** Perform row operation on the augmented matrix to solve for coefficients ***
   //       (loops are manually un-rolled for optimizations and increased speed)
   float scale;

   // Scale 1st line
   scale = 1.0f / M[0][0];
   M[0][0]  = 1.0f;
   M[0][1] *= scale;
   M[0][2] *= scale;
   M[0][3] *= scale;
   M[0][4] *= scale;

   // eliminate 1st coef on 2nd row
   M[1][4] -= M[1][0] * M[0][4];
   M[1][3] -= M[1][0] * M[0][3];
   M[1][2] -= M[1][0] * M[0][2];
   M[1][1] -= M[1][0] * M[0][1];
   M[1][0]  = 0.0f;

   // Scale 2nd row
   scale = 1.0f / M[1][1];
   M[1][1]  = 1.0f;
   M[1][2] *= scale;
   M[1][3] *= scale;
   M[1][4] *= scale;

   // eliminate 1st coef on 3rd row
   M[2][4] -= M[2][0] * M[0][4];
   M[2][3] -= M[2][0] * M[0][3];
   M[2][2] -= M[2][0] * M[0][2];
   M[2][1] -= M[2][0] * M[0][1];
   M[2][0]  = 0.0f;

   // eliminate 2nd coef on 3rd row
   M[2][4] -= M[2][1] * M[1][4];
   M[2][3] -= M[2][1] * M[1][3];
   M[2][2] -= M[2][1] * M[1][2];
   M[2][1]  = 0.0f;

   // Scale 3rd row
   scale = 1.0f / M[2][2];
   M[2][2]  = 1.0f;
   M[2][3] *= scale;
   M[2][4] *= scale;

   // eliminate 2nd coef on 1st row
   M[0][4] -= M[0][1] * M[1][4];
   M[0][3] -= M[0][1] * M[1][3];
   M[0][2] -= M[0][1] * M[1][2];
   M[0][1]  = 0.0f;

   // eliminate 3rd coef on 1st row
   M[0][4] -= M[0][2] * M[2][4];
   M[0][3] -= M[0][2] * M[2][3];
   M[0][2]  = 0.0f;

   // eliminate 3rd coef on 2nd row
   M[1][4] -= M[1][2] * M[2][4];
   M[1][3] -= M[1][2] * M[2][3];
   M[1][2]  = 0.0f;

   // eliminate 4th coef on 1st row
   M[0][4] -= M[0][3] * M[3][4];
   M[0][3]  = 0.0f;

   // eliminate 4th coef on 2nd row
   M[1][4] -= M[1][3] * M[3][4];
   M[1][3]  = 0.0f;

   // eliminate 4th coef on 3rd row
   M[2][4] -= M[2][3] * M[3][4];
   M[2][3]  = 0.0f;

   X.P_5 = M[0][4];
   X.P_4 = M[1][4];
   X.P_3 = M[2][4];

   X.C_1 = M[3][4];

   X.V_4 = M[0][4] * 5.0f;
   X.V_3 = M[1][4] * 4.0f;
   X.V_2 = M[2][4] * 3.0f;
}