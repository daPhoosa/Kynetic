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


      private:
         float minLineLength,  feed, accel, invAccelX2;
         float arcErrorX2, arcErrorSq;

         float cx, cy, radius, angle, dA;

         int segmentCount, segmentNow;

         struct POINT_4D_t
         {
            float x, y, z, e;
         } now, last, delta;


   } blockSplitter;

   blockSplitterObject::blockSplitterObject()
   {
      setMinLength( 1.0f );
      setAcceleration( 1500.0f );
      setArcError( 0.01 );
   }

   void blockSplitterObject::setMinLength( float L )
   {
      minLineLength = L;
   }

   void blockSplitterObject::setAcceleration( float A )
   {
      accel = A - 1; // bias to insure length is slightly longer than accel distance
      invAccelX2 = 1.0f / (2.0f * accel);
   }

   void blockSplitterObject::setArcError( float E )
   {
      arcErrorX2 = E * 2.0f;
      arcErrorSq = E * E;
   }


   void blockSplitterObject::addLine( float X0, float Y0, float Z0, float E0, float X1, float Y1, float Z1, float E1, float feedRate )
   {
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

      last.x = X1;
      last.y = Y1;
      last.z = Z1;
      last.e = E1;

      delta.x = X1 - X0;
      delta.y = Y1 - Y0;
      delta.z = Z1 - Z0;
      delta.e = E1 - E0;

      float dX = X0 - cx;
      float dY = Y0 - cy;

      radius = sqrt( dX * dX + dY * dY );

      // add zero radius check here
      
      feed = min( feedRate, sqrt( accel * radius )); // limit to radial acceleration

      float lengthTarget = max( feed * feed * invAccelX2, minLineLength  );
      lengthTarget = min( 2.0f * sqrt( radius * arcErrorX2 + arcErrorSq), lengthTarget );

      float startAngle = atan2f( dY, dX );
      if(startAngle < 0.0f) startAngle += 6.2831853f; // force positive

      dX = X1 - cx;
      dY = Y1 - cy;

      float endAngle = atan2f( dY, dX );
      if( endAngle < 0.0f ) endAngle += 6.2831853f; // force positive

      float arcAngle;

      if( direction == 2 ) // CW arc
      {
         arcAngle = startAngle - endAngle;
      }
      else                 // CCW arc
      {
         arcAngle = endAngle - startAngle;
      }

      if(arcAngle < 0.0001f) // must be positive (also matching start and stop locations indicates full circle)
      {
         arcAngle += 6.2831853f;
      }

      length = arcAngle * radius;
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
            now.x += delta.x; // increment position
            now.y += delta.y;
            now.z += delta.z;
            now.e += delta.e;
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
