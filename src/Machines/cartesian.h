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


#ifdef MACHINE_TYPE_CARTESIAN


   class cartesian_machine_type
   {
      public:

         
         void invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c );
         void fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z );
         
         void startHome( bool xHome, bool yHome, bool zHome );
         void abortHome();
         bool executeHome();

         bool allHomeCompleted();
         

      private:

         void homeAxis(int & index, dStepper & motor, int endStopPin, int switchNoContact, float homeOffset, float velocity );
      
         int A_homeIndex, B_homeIndex, C_homeIndex;
         bool homingActive = false;
         bool homingComplete = false;

   } machine;
 
   
   void cartesian_machine_type::invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c )
   {
      a = x;
      b = y;
      c = z;
   }
   
   
   void cartesian_machine_type::fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z )
   {
      x = a;
      y = b;
      z = c;
   }
   
   
   void cartesian_machine_type::startHome( bool xHome, bool yHome, bool zHome )
   {
      if( xHome && A_homeIndex < 2 )   // ( only reset if 0 (never home) or 1 (home complete), otherwise homing is already in process )
      {
         A_homeIndex = 6;
         homingActive = true;
         homingComplete = false;
      } 

      if( yHome && B_homeIndex < 2 )  
      {
         B_homeIndex = 6;
         homingActive = true;
         homingComplete = false;
      }

      if( zHome && C_homeIndex < 2 ) 
      {
         C_homeIndex = 6;
         homingActive = true;
         homingComplete = false;
      }
   }


   void cartesian_machine_type::homeAxis(int & index, dStepper & motor, int endStopPin, int switchNoContact, float homeOffset, float velocity )
   {

   }


   bool cartesian_machine_type::executeHome()
   {

   }


   void cartesian_machine_type::abortHome()
   {
      if( A_homeIndex > 1 || B_homeIndex > 1 || C_homeIndex > 1 ) // only abort if at least one axis is actively homing
      {
         A_homeIndex = 0;
         B_homeIndex = 0;
         C_homeIndex = 0;
         homingActive = true; // set to true to force a final execute that sets the motors to zero vel
         homingComplete = false;
      }
   }


   bool cartesian_machine_type::allHomeCompleted()
   {
      return homingComplete;
   }  

#endif