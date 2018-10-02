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

#ifndef blockSplitter_h
   #define blockSplitter_h

   #include <arduino.h>

   class blockSplitterObject
   {
      public:
         blockSplitterObject();

         void setMinLength( float L );
         void setMaxLength( float L );
         void setAcceleration( float A );
         void setArcError( float E );

         void addLine( float X0, float Y0, float Z0, float E0, float X1, float Y1, float Z1, float E1, float feedRate );
         void addArc(  float X0, float Y0, float Z0, float E0, float X1, float Y1, float Z1, float E1, float feedRate, float centerX, float centerY, int direction );

         bool getNextSegment();

         float x();
         float y();
         float z();
         float e();
         float f();

         enum MOVE_TYPE_t
         {
            RAPID,
            LINEAR,
            ARC_CW,
            ARC_CCW
         } moveType;


      private:
         float minLineLength, maxLineLength;
         float feed, accel, invAccelX2;
         float arcDeviationX8;

         float cx, cy, radius, angle, dA;

         int segmentCount, segmentNow;

         struct POINT_4D_t
         {
            float x, y, z, e;
         } now, last, delta;


   } blockSplitter;

   blockSplitterObject::blockSplitterObject()
   {
      setMinLength( 2.0f );
      setMaxLength( 10.0f );
      setAcceleration( 1500.0f );
      setArcError( 0.02 );
   }

   void blockSplitterObject::setMinLength( float L )
   {
      minLineLength = L;
   }

   void blockSplitterObject::setMaxLength( float L )
   {
      maxLineLength = L;
   }

   void blockSplitterObject::setAcceleration( float A )
   {
      accel = A * .98; // bias to insure length is slightly longer than accel distance
      invAccelX2 = 1.0f / (2.0f * accel);
   }

   void blockSplitterObject::setArcError( float E )
   {
      arcDeviationX8 = E * 8.0f;
   }


   void blockSplitterObject::addLine( float X0, float Y0, float Z0, float E0, float X1, float Y1, float Z1, float E1, float feedRate )
   {
      moveType = LINEAR;
      segmentNow = 0;

      feed = feedRate;

      float lengthTarget = max( feed * feed * invAccelX2, minLineLength );
      
      now.x = X0;
      now.y = Y0;
      now.z = Z0;
      now.e = E0;

      last.x = X1;
      last.y = Y1;
      last.z = Z1;
      last.e = E1;

      delta.x = X1 - X0;
      delta.y = Y1 - Y0;
      delta.z = Z1 - Z0;
      delta.e = E1 - E0;

      float length = sqrtf( delta.x * delta.x + delta.y * delta.y + delta.z * delta.z );
      
      if( length > lengthTarget )
      {
         segmentCount = int(length / lengthTarget + 0.5f);

         float invLength = 1.0f / float(segmentCount);

         delta.x *= invLength;
         delta.y *= invLength;
         delta.z *= invLength;
         delta.e *= invLength;
      }
      else
      {
         segmentCount = 1;
      }

   }

   void blockSplitterObject::addArc( float X0, float Y0, float Z0, float E0, float X1, float Y1, float Z1, float E1, float feedRate, float centerX, float centerY, int direction )
   {
      segmentNow = 0;

      cx = centerX;
      cy = centerY;

      now.x = X0;
      now.y = Y0;
      now.z = Z0;
      now.e = E0;

      last.x = X1;
      last.y = Y1;
      last.z = Z1;
      last.e = E1;

      delta.x = X1 - X0;
      delta.y = Y1 - Y0;
      delta.z = Z1 - Z0;
      delta.e = E1 - E0;

      float rx = X0 - cx;
      float ry = Y0 - cy;

      radius = sqrtf( rx * rx + ry * ry );

      feed = min( feedRate, sqrtf( accel * radius )); // limit to radial acceleration

      if( radius < 0.01 )  // make arcs with very small radius a straight line to avoid a divide by zero
      {
         segmentCount = 1;
         return;
      }

      // Length is limited by both arc path deviation and acceleration.  Selects the smaller one.
      float pathDeviationLength = sqrtf( radius * arcDeviationX8); // sqrt( radius * arcDev * 8 ) ~= 2 * sqrt( 2 * radius * arcDev + arcDev^2 ) -- simplification to reduce computation
      float linearAccelDistance = max( feed * feed * invAccelX2, minLineLength ); // distance to come to a complete stop if at full speed (dont over populate at very low feed rates)
      float lengthTarget        = min( pathDeviationLength, linearAccelDistance ); 

      float arcAngle;

      angle = atan2f( ry, rx ); // start angle

      if( delta.x * delta.x + delta.y * delta.y < 0.0001f ) // coincident start/stop points indicate full circle
      {
         if( direction == 2 )
         {
            moveType = ARC_CW;
            arcAngle = -6.2831853f;  // CW - sign dictates direction
         }
         else
         {
            moveType = ARC_CCW;
            arcAngle = 6.2831853f;  // ccw
         }
      }
      else
      {  
         float angleEnd = atan2f( Y1 - cy, X1 - cx );

         arcAngle = angleEnd - angle;

         if( direction == 2 )  // CW direction
         {
            moveType = ARC_CW;
            if( arcAngle > 0.0f ) arcAngle -= 6.2831853f; // when rotating CW, start must be larger than end
         }
         else                 // CCW direction
         {
            moveType = ARC_CCW;
            if( arcAngle < 0.0f ) arcAngle += 6.2831853f; // when rotating CCW, end must be larger than start
         }
      }
      
      segmentCount = int( abs(arcAngle) / (lengthTarget / radius) + 1.0f );
      float invCount = 1.0f / float(segmentCount);

      dA = arcAngle * invCount;
      delta.z = (Z1 - Z0) * invCount;
      delta.e = (E1 - E0) * invCount;

      if( abs(delta.z) > 0.1f ) // verify helical moves do not exceed programmed feed rate
      {
         float flatLength    = abs( arcAngle * radius );
         float totalLength   = sqrtf( flatLength * flatLength + delta.z * delta.z );
         float idealMoveTime = totalLength / feedRate;
         float flatMoveTime  = flatLength  / feed;

         if( idealMoveTime > flatMoveTime )
         {
            feed = flatLength / idealMoveTime;
         }
      }
   }

   bool blockSplitterObject::getNextSegment()
   {
      if( segmentNow < segmentCount )
      {
         segmentNow++;

         if( segmentNow == segmentCount )
         {
            now.x = last.x; // use end point for last segement
            now.y = last.y;
            now.z = last.z;
            now.e = last.e;
         }
         else
         {
            switch( moveType )
            {
               case RAPID:
               case LINEAR:
                  now.x += delta.x; // increment position
                  now.y += delta.y;
                  now.z += delta.z;
                  now.e += delta.e;
                  break;

               case ARC_CW:
               case ARC_CCW:
                  angle += dA;
                  now.x = radius * cosf( angle ) + cx;
                  now.y = radius * sinf( angle ) + cy;
                  now.z += delta.z;
                  now.e += delta.e; 
                  break;
            }
            
         }
         return true;
      }
      return false;
   }

   float blockSplitterObject::x()
   {
      return now.x;
   }

   float blockSplitterObject::y()
   {
      return now.y;
   }

   float blockSplitterObject::z()
   {
      return now.z;
   }

   float blockSplitterObject::e()
   {
      return now.e;
   }

   float blockSplitterObject::f()
   {
      return feed;
   }

#endif