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
         
         void invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c );
         void fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z );
         
         void startHome( bool xHome, bool yHome, bool zHome );
         void abortHome();
         bool executeHome();

         bool allHomeCompleted();
         

      private:
      
         bool actuatorPos( const float & x1, const float & y1, const float & x2, const float & y2, const float & z, float & result );
         float square( float x );
         
         void homeAxis(int & index, stepperMotor & motor, int endStopPin, int switchNoContact, float homeOffset, float velocity );

         int A_homeIndex, B_homeIndex, C_homeIndex;
         bool homingActive = false;
         bool homingComplete = false;
         
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


   void delta_machine_type::invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c )
   {
      // an axis that is asked to go to an unreachable location will fail silently and remain at its old location

      float result;

      if( actuatorPos( A_TowerX, A_TowerY, x, y, z, result ) )
      {
         a = result;
      }

      if( actuatorPos( B_TowerX, B_TowerY, x, y, z, result ) )
      {
         b = result;
      }

      if( actuatorPos( C_TowerX, C_TowerY, x, y, z, result ) )
      {
         c = result;
      }
   }
   

   bool delta_machine_type::actuatorPos( const float & x1, const float & y1, const float & x2, const float & y2, const float & z, float & result )
   {
      // returns true if the requested location is reachable
      
      float dx_Sq = square( x1 - x2 );
      float dy_Sq = square( y1 - y2 );
      float dz_Sq = armLengthSq - ( dx_Sq + dy_Sq );
      
      if( dz_Sq > minArmHeightSq )
      {
         result = sqrtf( dz_Sq ) + z;
         return true;
      }

      return false; // location outside of reachable area
   }


   void delta_machine_type::fwdKinematics( const float & A_Actuator, const float & B_Actuator, const float & C_Actuator, float & x, float & y, float & z )
   {
      // adapted from https://github.com/Smoothieware/Smoothieware/blob/master/src/modules/robot/arm_solutions/LinearDeltaSolution.cpp
      // from http://en.wikipedia.org/wiki/Circumscribed_circle#Barycentric_coordinates_from_cross-_and_dot-products
      // based on https://github.com/ambrop72/aprinter/blob/2de69a/aprinter/printer/DeltaTransform.h#L81

      Vec3 tower1( A_TowerX, A_TowerY, A_Actuator );
      Vec3 tower2( B_TowerX, B_TowerY, B_Actuator );
      Vec3 tower3( C_TowerX, C_TowerY, C_Actuator );

      Vec3 s12 = VectorSub( tower1, tower2 );
      Vec3 s23 = VectorSub( tower2, tower3 );
      Vec3 s13 = VectorSub( tower1, tower3 );

      Vec3 normal = VecCrossProd( s12, s23 );

      float magsq_s12 = VecMagSq( s12 );
      float magsq_s23 = VecMagSq( s23 );
      float magsq_s13 = VecMagSq( s13 );

      float inv_nmag_sq = 1.0F / VecMagSq( normal );
      float q = 0.5F * inv_nmag_sq;

      float a = q * magsq_s23 * VecDotProd( s12, s13 );
      float b = q * magsq_s13 * VecDotProd( s12, s23 ) * -1.0F; // negate because we use s12 instead of s21
      float c = q * magsq_s12 * VecDotProd( s13, s23 );

      Vec3 circumcenter( A_TowerX   * a + B_TowerX   * b + C_TowerX   * c,
                         A_TowerY   * a + B_TowerY   * b + C_TowerY   * c,
                         A_Actuator * a + B_Actuator * b + C_Actuator * c );

      float r_sq = 0.5F * q * magsq_s12 * magsq_s23 * magsq_s13;
      float dist = sqrtf(inv_nmag_sq * (armLengthSq - r_sq));

      Vec3 cartesian = VectorSub( circumcenter, VectorMul( normal, dist ));

      x = cartesian.x;
      y = cartesian.y;
      z = cartesian.z;
   }


   float delta_machine_type::square( float x )
   {
      return x * x;
   }   


   void delta_machine_type::startHome( bool xHome, bool yHome, bool zHome )
   {
      if( A_homeIndex < 2 ) A_homeIndex = 6; // for delta, allways home all motors at once.  Don't reset motors that are already homing
      if( B_homeIndex < 2 ) B_homeIndex = 6; // ( only reset if 0 (never home) or 1 (home complete), otherwise homing is in process )
      if( C_homeIndex < 2 ) C_homeIndex = 6;
      homingActive = true;
      homingComplete = false;
   }


   bool delta_machine_type::allHomeCompleted()
   {
      return homingComplete;
   }


   void delta_machine_type::abortHome()
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


   bool delta_machine_type::executeHome()
   {
      if(homingActive)
      {
         if( A_homeIndex > 4 ) {
            homeAxis(A_homeIndex, A_motor, X_ENDSTOP_PIN, X_ENDSTOP_NO_CONTACT, A_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
         }else{
            homeAxis(A_homeIndex, A_motor, X_ENDSTOP_PIN, X_ENDSTOP_NO_CONTACT, A_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
         }

         if( B_homeIndex > 4 ) {
            homeAxis(B_homeIndex, B_motor, Y_ENDSTOP_PIN, Y_ENDSTOP_NO_CONTACT, B_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
         }else{
            homeAxis(B_homeIndex, B_motor, Y_ENDSTOP_PIN, Y_ENDSTOP_NO_CONTACT, B_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
         }

         if( C_homeIndex > 4 ) {
            homeAxis(C_homeIndex, C_motor, Z_ENDSTOP_PIN, Z_ENDSTOP_NO_CONTACT, C_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
         }else{
            homeAxis(C_homeIndex, C_motor, Z_ENDSTOP_PIN, Z_ENDSTOP_NO_CONTACT, C_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
         }

         if( A_homeIndex == 1 && B_homeIndex == 1 && C_homeIndex == 1 ) // not done going home until all axis are done
         {
            homingActive = false;
            homingComplete = true;
            return true; // returns true once all axis are at home
         }
         else
         {
            return false; // not at home
         }
      }
      return false;
   }
   
   
   void delta_machine_type::homeAxis(int & index, stepperMotor & motor, int endStopPin, int switchNoContact, float homeOffset, float velocity )
   {
      float speed = motor.getSpeed(); 
      
      switch(index)
      {
         case 6 : // fast advance
         case 4 : // slow advance
            if( digitalRead( endStopPin ) == switchNoContact )
            {
               speed += MACHINE_VEL_STEP_Z;
               if( speed > velocity ) speed = velocity;
               
               motor.setSpeed( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               motor.setPosition( homeOffset ); // switched has been activated
               index--;
            }
          
         case 5 : // fast retract
         case 3 : // slow retract
            if( motor.getPositionMM() > homeOffset - SLOW_HOME_DIST )
            {
               speed -= MACHINE_VEL_STEP_Z;
               if( speed < -velocity ) speed = -velocity;
               
               motor.setSpeed( speed );  // back away from switch
               break;
            }
            else
            {
               index--;
            }
            
         case 2 : // decelerate to zero after slow retract
            speed += MACHINE_VEL_STEP_Z;
            
            if( speed < 0.0f )
            {
               motor.setSpeed( speed );  // decelerate
               break;
            }
            else
            {
               index = 1;
            }

         case 1 : // hold zero speed
         case 0 : 
            motor.setSpeed( 0.0f );
            break;
      }
   }
   
#endif