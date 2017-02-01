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


#ifdef MACHINE_TYPE_COREXY


   class coreXY_machine_type
   {
      public:
         
         void init();
         
         void invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c );
         
         void fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z );
         

      private:
      

   } machine;
   
   
   void coreXY_machine_type::init()
   {
   }
   
   
   void coreXY_machine_type::invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c )
   {
      a = x + y;
      b = x - y;
      c = z;
   }
   
   
   void coreXY_machine_type::fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z )
   {
      x = ( a + b ) * 0.5f;
      y = ( a - b ) * 0.5f;
      z = c;
   }
   
   
#endif