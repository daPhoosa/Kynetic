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


#ifdef MACHINE_TYPE_DELTA


   class delta_machine_type
   {
      public:
         
         //void init();
         
         void invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c );
         
         //void fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z );
         
         bool home( bool xHome, bool yHome, bool zHome );
         

      private:
      
         bool computeDeltaPos( const float & x1, const float & y1, const float & x2, const float & y2, const float & z, float & result );
         float square( float x );
         
         bool startHome( bool xHome, bool yHome, bool zHome );
         void homeAxis(int & index, uStepper & motor, int endStopPin, int switchNoContact, float homeOffset, float velocity );

         int A_homeIndex, B_homeIndex, C_homeIndex;
         bool homingActive = false;
         
         //float A_TowerX, A_TowerY, B_TowerX, B_TowerY, C_TowerX, C_TowerY;
         //float minArmHeightSq, armLengthSq;
         
         const float sin60 = 0.86602540378f;
         const float cos60 = 0.5f;
         
         const float A_TowerX = -sin60 * DELTA_ARM_RADIUS;
         const float A_TowerY = -cos60 * DELTA_ARM_RADIUS;
         const float B_TowerX =  sin60 * DELTA_ARM_RADIUS;
         const float B_TowerY = -cos60 * DELTA_ARM_RADIUS;
         const float C_TowerX =  0.0f;
         const float C_TowerY =  DELTA_ARM_RADIUS;
         
         const float armLengthSq = DELTA_ARM_LENGTH * DELTA_ARM_LENGTH;
         
         const float minArmHeightSq = square( sin( DELTA_MIN_ARM_ANGLE * 0.0174533f ) * DELTA_ARM_LENGTH );  // 

   } machine;

   /*
   void delta_machine_type::init()
   {
      const float sin60 = 0.86602540378f;
      const float cos60 = 0.5f;
      
      A_TowerX = -sin60 * DELTA_ARM_RADIUS;
      A_TowerY = -cos60 * DELTA_ARM_RADIUS;
      B_TowerX =  sin60 * DELTA_ARM_RADIUS;
      B_TowerY = -cos60 * DELTA_ARM_RADIUS;
      C_TowerX =  0.0f;
      C_TowerY =  DELTA_ARM_RADIUS;
      
      armLengthSq = DELTA_ARM_LENGTH * DELTA_ARM_LENGTH;
      
      minArmHeightSq = square( sin( DELTA_MIN_ARM_ANGLE * 0.0174533f ) * DELTA_ARM_LENGTH );  // 
   }
   */
   
   void delta_machine_type::invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c )
   {
      // an axis that is asked to go to an unreachable location will fail silently and remain at its old location

      float result;
      
      if( computeDeltaPos( A_TowerX, A_TowerY, x, y, z, result ) )
      {
         a = result;
      }

      if( computeDeltaPos( B_TowerX, B_TowerY, x, y, z, result ) )
      {
         b = result;
      }

      if( computeDeltaPos( B_TowerX, B_TowerY, x, y, z, result ) )
      {
         c = result;
      }

   }
   
   /*
   void delta_machine_type::fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z )
   {
      // todo...
   }
   */


   bool delta_machine_type::startHome( bool xHome, bool yHome, bool zHome )
   {
      A_homeIndex = 5; // for delta, allways home all motors at once
      B_homeIndex = 5;
      C_homeIndex = 5;
      homingActive = true;
   }


   bool delta_machine_type::executeHome()
   {
      
      if(homingActive)
      {
         if( A_homeIndex > 3 ) {
            homeAxis(A_homeIndex, A_motor, X_MAX_ENDSTOP_PIN, X_MAX_ENDSTOP_NO_CONTACT, A_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
         }else{
            homeAxis(A_homeIndex, A_motor, X_MAX_ENDSTOP_PIN, X_MAX_ENDSTOP_NO_CONTACT, A_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
         }

         if( B_homeIndex > 3 ) {
            homeAxis(B_homeIndex, B_motor, Y_MAX_ENDSTOP_PIN, Y_MAX_ENDSTOP_NO_CONTACT, B_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
         }else{
            homeAxis(B_homeIndex, B_motor, Y_MAX_ENDSTOP_PIN, Y_MAX_ENDSTOP_NO_CONTACT, B_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
         }

         if( C_homeIndex > 3 ) {
            homeAxis(C_homeIndex, C_motor, Z_MAX_ENDSTOP_PIN, Z_MAX_ENDSTOP_NO_CONTACT, C_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
         }else{
            homeAxis(C_homeIndex, C_motor, Z_MAX_ENDSTOP_PIN, Z_MAX_ENDSTOP_NO_CONTACT, C_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
         }

         if( A_homeIndex || B_homeIndex || C_homeIndex ) // not done going home until all axis are done
         {
              return false; // not at home
         }
         else
         {
            homingActive = false;
         }
      }
 
      return true; // returns true once all axis are at home
   }
   
   
   void delta_machine_type::homeAxis(int & index, uStepper & motor, int endStopPin, int switchNoContact, float homeOffset, float velocity )
   {
      float speed = motor.getSpeed(); 
      
      switch(index)
      {
         case 5 : // fast advance
         case 3 : // slow advance
            if( digitalRead( endStopPin ) == switchNoContact )
            {
               speed += MACHINE_VEL_STEP;
               if( speed > velocity ) speed = velocity;
               
               motor.setSpeed( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               motor.setPosition( homeOffset ); // switched has been activated
               index--;
            }
          
         case 4 : // fast retract
         case 2 : // slow retract
            if( motor.getPositionMM() > homeOffset - SLOW_HOME_DIST )
            {
               speed -= MACHINE_VEL_STEP;
               if( speed < -velocity ) speed = -velocity;
               
               motor.setSpeed( speed );  // back away from switch
               break;
            }
            else
            {
               index--;
            }
            
         case 1 : // decelerate to zero after slow retract
            speed += MACHINE_VEL_STEP;
            
            if( speed < 0.0f )
            {
               motor.setSpeed( speed );  // decelerate
               break;
            }

         case 0 : // hold zero
            motor.setSpeed( 0.0f );
            break;
      }
   }
   
   
   bool delta_machine_type::computeDeltaPos( const float & x1, const float & y1, const float & x2, const float & y2, const float & z, float & result )
   {
      // returns true if the requested location is reachable
      
      static float dx_Sq, dy_Sq, dz_Sq;
      
      dx_Sq = square( x1 - x2 );
      
      dy_Sq = square( y1 - y2 );
      
      dz_Sq = armLengthSq - ( dx_Sq + dy_Sq );
      
      if( dz_Sq > minArmHeightSq )
      {
         result = sqrt( dz_Sq ) + z;
         return true;
      }

      return false; // location outside of reachable area
   }


   float delta_machine_type::square( float x )
   {
      return x * x;
   }   

   
#endif