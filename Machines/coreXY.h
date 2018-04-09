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

         void invKinematics( const float & x, const float & y, const float & z, float & a, float & b, float & c );
         void fwdKinematics( const float & a, const float & b, const float & c, float & x, float & y, float & z );

         void startHome( bool xHome, bool yHome, bool zHome );
         void abortHome();
         bool executeHome();

         bool homingActive();


      private:

         float getVelX();
         float getVelY();
         float getVelZ();

         void  setVelX( float velX );
         void  setVelY( float velY );
         void  setVelZ( float vel );

         float getPosX();
         float getPosY();
         float getPosZ();

         void  setPosX( float pos );
         void  setPosY( float pos );
         void  setPosZ( float pos );

         void homeAxisX( float velocity );
         void homeAxisY( float velocity );
         void homeAxisZ( float velocity );

         float inline dX( const float & x );
         float inline dY( const float & y );
         float inline dZ( const float & z );

         int X_homeIndex, Y_homeIndex, Z_homeIndex;
         bool homingNow;

   } machine;


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


   void coreXY_machine_type::startHome( bool xHome, bool yHome, bool zHome )
   {
      if( xHome ) 
      {
         Serial.println("START X HOME");
         X_homeIndex = 6;
         homingNow = true;
      }

      if( yHome )
      {
         Serial.println("START Y HOME");
         Y_homeIndex = 6;
         homingNow = true;
      }

      if( zHome )
      {
         Serial.println("START Z HOME");
         Z_homeIndex = 6;
         homingNow = true;
      }
   }


   void coreXY_machine_type::homeAxisX( float velocity )
   {
      float speed = getVelX();

      switch( X_homeIndex )
      {
         case 6 : // fast advance
         case 4 : // slow advance
            if( digitalRead( X_ENDSTOP_PIN ) == X_ENDSTOP_NO_CONTACT )
            {
               speed += dX(MACHINE_VEL_STEP_XY);
               if( dX(speed) > velocity ) speed = dX(velocity);

               setVelX( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               Serial.print(X_homeIndex); Serial.print(" X Advance - v:"); Serial.print(speed); Serial.print("  t:"); Serial.println(millis());
               setPosX( X_HOME_OFFSET ); // switched has been activated
               X_homeIndex--;
            }

         case 5 : // fast retract
         case 3 : // slow retract
            if( dX(getPosX()) > X_HOME_OFFSET - dX(SLOW_HOME_DIST) )
            {
               speed -= dX(MACHINE_VEL_STEP_XY);
               if( dX(speed) < -velocity ) speed = dX(-velocity);

               setVelX( speed );  // back away from switch
               break;
            }
            else
            {
               Serial.print(X_homeIndex); Serial.print(" X Retract - v:"); Serial.print(speed); Serial.print("  t:"); Serial.println(millis());
               X_homeIndex--;
            }

         case 2 : // decelerate to zero after slow retract
            speed += dX(MACHINE_VEL_STEP_XY);

            if( dX(speed) < 0.0f )
            {
               setVelX( speed );  // decelerate
               break;
            }
            else
            {
               Serial.print("X Home Done  t:"); Serial.println(millis());
               X_homeIndex = 1;
            }

         case 1 : // hold zero speed
         case 0 :
            setVelX( 0.0f );
            break;
      }
   }


   void coreXY_machine_type::homeAxisY( float velocity )
   {
      float speed = getVelY();

      switch( Y_homeIndex )
      {
         case 6 : // fast advance
         case 4 : // slow advance
            if( digitalRead( Y_ENDSTOP_PIN ) == Y_ENDSTOP_NO_CONTACT )
            {
               speed += dY(MACHINE_VEL_STEP_XY);
               if( dY(speed) > velocity ) speed = dY(velocity);

               setVelY( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               Serial.print(Y_homeIndex); Serial.print(" Y Advance - v:"); Serial.print(speed); Serial.print("  t:"); Serial.println(millis());
               setPosY( Y_HOME_OFFSET ); // switched has been activated
               Y_homeIndex--;
            }

         case 5 : // fast retract
         case 3 : // slow retract
            if( dY(getPosY()) > Y_HOME_OFFSET - dY(SLOW_HOME_DIST) )
            {
               speed -= dY(MACHINE_VEL_STEP_XY);
               if( dY(speed) < -velocity ) speed = dY(-velocity);

               setVelY( speed );  // back away from switch
               break;
            }
            else
            {
               Serial.print(Y_homeIndex); Serial.print(" Y Retract - v:"); Serial.print(speed); Serial.print("  t:"); Serial.println(millis());
               Y_homeIndex--;
            }

         case 2 : // decelerate to zero after slow retract
            speed += dY(MACHINE_VEL_STEP_XY);

            if( dY(speed) < 0.0f )
            {
               setVelY( speed );  // decelerate
               break;
            }
            else
            {
               Serial.print("Y Home Done  t:"); Serial.println(millis());
               Y_homeIndex = 1;
            }

         case 1 : // hold zero speed
         case 0 :
            setVelY( 0.0f );
            break;
      }
   }


   void coreXY_machine_type::homeAxisZ( float velocity )
   {
      float speed = getVelZ();

      switch(Z_homeIndex)
      {
         case 6 : // fast advance
         case 4 : // slow advance
            if( digitalRead( Z_ENDSTOP_PIN ) == Z_ENDSTOP_NO_CONTACT )
            {
               speed += dZ(MACHINE_VEL_STEP_Z);
               if( dZ(speed) > velocity ) speed = dZ(velocity);

               setVelZ( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               Serial.print(Z_homeIndex); Serial.print(" Z Advance - v:"); Serial.print(speed); Serial.print("  t:"); Serial.println(millis());
               setPosZ( Z_HOME_OFFSET ); // switch has been activated
               Z_homeIndex--;
            }

         case 5 : // fast retract
         case 3 : // slow retract
            if( dZ(getPosZ()) > Z_HOME_OFFSET - dZ(SLOW_HOME_DIST) )
            {
               speed -= dZ(MACHINE_VEL_STEP_Z);
               if( dZ(speed) < -velocity ) speed = dZ(-velocity);

               setVelZ( speed );  // back away from switch
               break;
            }
            else
            {
               Serial.print(Z_homeIndex); Serial.print(" Z Retract - v:"); Serial.print(speed); Serial.print("  t:"); Serial.println(millis());
               Z_homeIndex--;
            }

         case 2 : // decelerate to zero after slow retract
            speed += dZ(MACHINE_VEL_STEP_XY);

            if( dZ(speed) < 0.0f )
            {
               setVelZ( speed );  // decelerate
               break;
            }
            else
            {
               Serial.print("Z Home Done  t:"); Serial.println(millis());
               Z_homeIndex = 1;
            }

         case 1 : // hold zero speed
         case 0 :
            setVelZ( 0.0f );
            break;
      }
   }


   bool coreXY_machine_type::executeHome()
   {
      if( homingNow )
      {
         if( X_homeIndex > 4 ) {
            homeAxisX( FAST_HOME_VEL );
         }else{
            homeAxisX( SLOW_HOME_VEL );
         }

         if( Y_homeIndex > 4 ) {
            homeAxisY( FAST_HOME_VEL );
         }else{
            homeAxisY( SLOW_HOME_VEL );
         }

         if( Z_homeIndex > 4 ) {
            homeAxisZ( FAST_HOME_VEL );
         }else{
            homeAxisZ( SLOW_HOME_VEL );
         }

         if( X_homeIndex < 2 && Y_homeIndex < 2 && Z_homeIndex < 2 ) // not done going home until all axis are done
         {
            homingNow = false;
            return true; // returns true a single time once all axis are at home
         }
      }
      return false;
   }


   void coreXY_machine_type::abortHome()
   {
      if( X_homeIndex > 1 || Y_homeIndex > 1 || Z_homeIndex > 1 ) // only abort if at least one axis is actively homing
      {
         if( X_homeIndex > 1 ) X_homeIndex = 0; // invalidate any axis that is currenly homing
         if( Y_homeIndex > 1 ) Y_homeIndex = 0;
         if( Z_homeIndex > 1 ) Z_homeIndex = 0;
         homingNow = true; // set to true to force a final execute that sets the motors to zero vel
      }
   }


   bool coreXY_machine_type::homingActive()
   {
      return homingNow;
   }


   // GET VELOCITY
   float coreXY_machine_type::getVelX()
   {
      return ( A_motor.getSpeed() + B_motor.getSpeed() ) * 0.5f;
   }

   float coreXY_machine_type::getVelY()
   {
      return ( A_motor.getSpeed() - B_motor.getSpeed() ) * 0.5f;
   }

   float coreXY_machine_type::getVelZ()
   {
      return C_motor.getSpeed();
   }


   // SET VELOCITY
   void  coreXY_machine_type::setVelX( float velX )
   {
      float velY = getVelY();

      A_motor.setSpeed( velX + velY );
      B_motor.setSpeed( velX - velY );
   }

   void  coreXY_machine_type::setVelY( float velY )
   {
      float velX = getVelX();

      A_motor.setSpeed( velX + velY );
      B_motor.setSpeed( velX - velY );
   }

   void  coreXY_machine_type::setVelZ( float vel )
   {
      C_motor.setSpeed( vel );
   }


   // GET POSITION
   float coreXY_machine_type::getPosX()
   {
      return ( A_motor.getPositionMM() + B_motor.getPositionMM() ) * 0.5f;
   }

   float coreXY_machine_type::getPosY()
   {
      return ( A_motor.getPositionMM() - B_motor.getPositionMM() ) * 0.5f;
   }

   float coreXY_machine_type::getPosZ()
   {
      return C_motor.getPositionMM();
   }


   // SET POSITION
   void coreXY_machine_type::setPosX( float posX )
   {
      float y = getPosY();

      A_motor.setPosition( posX + y );
      B_motor.setPosition( posX - y );
   }

   void coreXY_machine_type::setPosY( float posY )
   {
      float x = getPosX();

      A_motor.setPosition( x + posY );
      B_motor.setPosition( x - posY );
   }

   void coreXY_machine_type::setPosZ( float pos )
   {
      C_motor.setPosition( pos );
   }


   // HOMING DIRECTION
   float inline coreXY_machine_type::dX( const float & x )
   {
      return x * X_HOME_DIRECTION;
   }

   float inline coreXY_machine_type::dY( const float & y )
   {
      return y * Y_HOME_DIRECTION;
   }

   float inline coreXY_machine_type::dZ( const float & z )
   {
      return z * Z_HOME_DIRECTION;
   }


#endif