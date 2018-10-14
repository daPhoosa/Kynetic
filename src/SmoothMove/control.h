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


void SmoothMove::setPosition( float t_x, float t_y, float t_z )
{
   setPosX( t_x );
   setPosY( t_y );
   setPosZ( t_z );   
}


void SmoothMove::setPosition( float t_x, float t_y, float t_z, float t_e )
{
   setPosX( t_x );
   setPosY( t_y );
   setPosZ( t_z );   
   setPosE( t_e ); 
}


void SmoothMove::setPosX( float t_x )
{
   if( !motionStopped ) motionStopped = true;

   X_end = t_x; // no queued blocks, so end point equals start point
}


void SmoothMove::setPosY( float t_y )
{
   if( !motionStopped ) motionStopped = true;

   Y_end = t_y; // no queued blocks, so end point equals start point
}


void SmoothMove::setPosZ( float t_z )
{
   if( !motionStopped ) motionStopped = true;

   Z_end = t_z; // no queued blocks, so end point equals start point
}


void SmoothMove::setPosE( float t_e )
{
   if( !motionStopped ) motionStopped = true;

   extrudeProgPos = t_e;
   extrudeMachPos = t_e;
}


float SmoothMove::setMotionRateOverride(  float scale )
{
   motionFeedOverride = constrain( scale, 0.1f, 2.0f );
   return motionFeedOverride;
}


float SmoothMove::setExtrudeRateOverride( float scale )
{
   extrudeRateOverride = constrain( scale, 0.1f, 2.0f );
   return extrudeRateOverride;
}


void SmoothMove::setExrudeAccel( float accel )
{
   extrudeAccel = accel; // [mm/s^2]
}

float SmoothMove::getSpeed()
{
   return velocityNow;
}

void SmoothMove::junctionSmoothingOff()
{
   pathSmoothingOff = true;
}

void SmoothMove::junctionSmoothingOn()
{
   pathSmoothingOff = false;
}


void SmoothMove::setLookAheadTime(int timeMS )
{
   lookAheadTimeMin = max( 5, timeMS ) * 1000UL; // time in microseconds
}


int SmoothMove::getBlockCount()
{
   return blockCount;
}


void SmoothMove::setParamXY( float accel, float maxVel )
{
   maxAccel_XY         = accel;
   accelInverse_XY     = 1.0f / accel;
   accelInverseHalf_XY = 0.5f * accelInverse_XY;
   accelDouble_XY      = 2.0f * accel;

   maxVel_XY = maxVel;
}


void SmoothMove::setParamZ( float accel, float maxVel )
{
   maxAccel_Z         = accel;
   accelInverse_Z     = 1.0f / accel;
   accelInverseHalf_Z = 0.5f * accelInverse_Z;
   accelDouble_Z      = 2.0f * accel;

   maxVel_Z = maxVel;
}


void SmoothMove::setCornerRounding( float _cornerRounding )
{
   cornerRoundDist     = max( _cornerRounding, 0.001f );
   cornerRoundDistSq   = cornerRoundDist * cornerRoundDist;
}


void SmoothMove::setJunctionVelRad( float t_r )
{
   junctionRadius     = max( t_r, 0.001f );
   junctionRadiusSq   = t_r * t_r;
}

void SmoothMove::setJunctionDeviation( float junkDev )
{
   junctionDeviation = junkDev;

   setJunctionVelRad( junkDev / 0.4f );
}


void SmoothMove::setExtrudeVelocityAdvance( float advance )  // units of (mm / mm/s x 1000)
{
   extrudeVelocityAdvance = advance / 1000.0f;
}