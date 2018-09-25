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

         void addLine( float startX, float startY, float startZ, float endX, float endY, float endZ, float feedRate );
         void addArc(  float startX, float startY, float startZ, float endX, float endY, float endZ, float feedRate, float centerX, float centerY);

         bool getNextSegment();

         float x();
         float y();
         float z();
         float f();


      private:
         float minLineLength, maxLineLength;

         int segmentCount, segmentNow;

         struct 3D_POINT_t
         {
            float x, y, z;
         } s, e, d, n;


   } blockSplitter;

   blockSplitterObject::blockSplitterObject()
   {
      minLineLength = 1.0f;
   }

   void blockSplitterObject::AddLine( float startX, float startY, float startZ, float endX, float endY, float endZ, float feedRate )
   {
      segmentNow = 0;
      
      s.x = startX;
      s.y = startY;
      s.z = startZ;

      e.x = endX;
      e.y = endY;
      e.z = endZ;

      d.x = e.x - s.x;
      d.y = e.y - s.y;
      d.z = e.z - s.z;

      float length = sqrtf( d.x * d.x + d.y * d.y + d.z * d.z );
      
      if( length > minLineLength )
      {
         segmentCount = int(length / minLineLength + 0.5f);

         float invLength = 1.0f / float(segmentCount);

         d.x *= invLength;
         d.y *= invLength;
         d.z *= invLength;
      }
      else
      {
         segmentCount = 1;
      }

   }

   void blockSplitterObject::addArc(  float startX, float startY, float startZ, float endX, float endY, float endZ, float feedRate, float centerX, float centerY)
   {

   }

#endif
