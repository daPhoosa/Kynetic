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
         
         void init();
         
         void invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c );
         
         //void fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z );
         
         bool home( bool & xHome, bool & yHome, bool & zHome );
         

      private:
      
         bool computeDeltaPos( const float & x1, const float & y1, const float & x2, const float & y2, const float & z, float & result );
         float square( float x );
         
         float A_TowerX, A_TowerY, B_TowerX, B_TowerY, C_TowerX, C_TowerY;
         float minArmHeightSq, armLengthSq;

   } machine;

   
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
      
      minArmHeightSq = square( 0.258819f * DELTA_ARM_LENGTH );  // sin(15deg) * armLength
   }
   
   
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
   
   bool delta_machine_type::home( bool & xHome, bool & yHome, bool & zHome )
   {
      int static A_index = B_index = C_index = 0;
      
      if( xHome && !A_index ) A_index = 5; // reset index
      if( yHome && !B_index ) B_index = 5;
      if( zHome && !C_index ) C_index = 5;
      
      if( A_index > 3 ) {
         homeAxis(A_home, A_motor, A_ENDSTOP_PIN, A_ENDSTOP_NO_CONTACT, A_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
      }else{
         homeAxis(A_home, A_motor, A_ENDSTOP_PIN, A_ENDSTOP_NO_CONTACT, A_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
      }

      if( B_index > 3 ) {
         homeAxis(B_home, B_motor, B_ENDSTOP_PIN, B_ENDSTOP_NO_CONTACT, B_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
      }else{
         homeAxis(B_home, B_motor, B_ENDSTOP_PIN, B_ENDSTOP_NO_CONTACT, B_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
      }

      if( C_index > 3 ) {
         homeAxis(C_home, C_motor, C_ENDSTOP_PIN, C_ENDSTOP_NO_CONTACT, C_MOTOR_HOME_OFFSET, FAST_HOME_VEL);
      }else{
         homeAxis(C_home, C_motor, C_ENDSTOP_PIN, C_ENDSTOP_NO_CONTACT, C_MOTOR_HOME_OFFSET, SLOW_HOME_VEL);
      }

      if( A_index || B_index || C_index ) return false;
      
      return true; // returns true once all axis are at home
   }
   
   
   bool delta_machine_type::homeAxis(int & index, uStepper* motor, int endStopPin, int switchNoContact, float homeOffset, float velocity )
   {
      float speed = motor->getSpeed(); 
      
      switch(index)
      {
         case 5 :
         case 3 :
            if( digitalRead( endStopPin ) == switchNoContact )
            {
               speed += MACHINE_VEL_STEP;
               if( speed > velocity ) speed = velocity;
               
               motor->setSpeed( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               motor->setPosition( homeOffset );
               index--;
            }
          
         case 4 :
         case 2 :
            if( motor->getPositionMM() > homeOffset - SLOW_HOME_DIST )
            {
               speed -= MACHINE_VEL_STEP;
               if( speed < -velocity ) speed = -velocity;
               
               motor->setSpeed( speed );  // back away from first contact
               break;
            }
            else
            {
               index--;
            }
            
         case 1 :
            speed += MACHINE_VEL_STEP;
            
            if( speed < 0.0f )
            {
               motor->setSpeed( speed );  // decelerate
               break;
            }

         case 0 :
            motor->setSpeed( 0.0f );
            break;
      }

      if( index ) return false;
      
      return true; // home complete 
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