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

         bool allHomeCompleted();
         

      private:

         float getVelX();
         float getVelY();
         void  setVelX( float velX );
         void  setVelY( float velY );

         float getPosX();
         float getPosY();
         void  setPosX( float pos );
         void  setPosY( float pos );

         void homeAxisX( float velocity );
         void homeAxisY( float velocity );
         void homeAxisZ( float velocity );
      
         int X_homeIndex, Y_homeIndex, Z_homeIndex;
         bool homingActive = false;
         bool homingComplete = false;

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
      if( xHome && X_homeIndex < 2 )   // ( only reset if 0 (never home) or 1 (home complete), otherwise homing is already in process )
      {
         X_homeIndex = 6;
         homingActive = true;
         homingComplete = false;
      } 

      if( yHome && Y_homeIndex < 2 )  
      {
         Y_homeIndex = 6;
         homingActive = true;
         homingComplete = false;
      }

      if( zHome && Z_homeIndex < 2 ) 
      {
         Z_homeIndex = 6;
         homingActive = true;
         homingComplete = false;
      }
   }


   void coreXY_machine_type::homeAxisX( float velocity )
   {
      float speed = getVelX(); 
      
      switch( X_homeIndex )
      {
         case 6 : // fast advance
         case 4 : // slow advance
            if( digitalRead( X_ENDSTOP_PIN ) == X_ENDSTOP_NO_CONTACT )  // check if endstop has NOT been touched
            {
               speed += MACHINE_VEL_STEP_XY * X_HOME_DIRECTION;
               if( speed * X_HOME_DIRECTION > velocity ) speed = velocity * X_HOME_DIRECTION;
               
               setVelX( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               setPosX( X_HOME_OFFSET ); // switched has been activated
               X_homeIndex --;
            }
          
         case 5 : // fast retract
         case 3 : // slow retract
            if( getPosX() * X_HOME_DIRECTION > X_HOME_OFFSET - SLOW_HOME_DIST * X_HOME_DIRECTION )
            {
               speed -= MACHINE_VEL_STEP_XY * X_HOME_DIRECTION;
               if( speed * X_HOME_DIRECTION < -velocity ) speed = -velocity * X_HOME_DIRECTION;
               
               setVelX( speed );  // back away from switch
               break;
            }
            else
            {
               X_homeIndex --;
            }
            
         case 2 : // decelerate to zero after slow retract
            speed += MACHINE_VEL_STEP_XY * X_HOME_DIRECTION;
            
            if( speed * X_HOME_DIRECTION < 0.0f )
            {
               setVelX( speed );  // decelerate
               break;
            }
            else
            {
               X_homeIndex  = 1;
            }

         case 1 : // hold zero speed, home set
         case 0 : // hold zero speed, home not set
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
            if( digitalRead( Y_ENDSTOP_PIN ) == Y_ENDSTOP_NO_CONTACT )  // check if endstop has NOT been touched
            {
               speed += MACHINE_VEL_STEP_XY * Y_HOME_DIRECTION;
               if( speed * Y_HOME_DIRECTION > velocity ) speed = velocity * Y_HOME_DIRECTION;
               
               setVelY( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               setPosY( Y_HOME_OFFSET ); // switched has been activated
               Y_homeIndex--;
            }
          
         case 5 : // fast retract
         case 3 : // slow retract
            if( getPosX() * Y_HOME_DIRECTION > Y_HOME_OFFSET - SLOW_HOME_DIST * Y_HOME_DIRECTION )
            {
               speed -= MACHINE_VEL_STEP_XY * Y_HOME_DIRECTION;
               if( speed * Y_HOME_DIRECTION < -velocity ) speed = -velocity * Y_HOME_DIRECTION;
               
               setVelY( speed );  // back away from switch
               break;
            }
            else
            {
               Y_homeIndex--;
            }
            
         case 2 : // decelerate to zero after slow retract
            speed += MACHINE_VEL_STEP_XY * Y_HOME_DIRECTION;
            
            if( speed * Y_HOME_DIRECTION < 0.0f )
            {
               setVelY( speed );  // decelerate
               break;
            }
            else
            {
               Y_homeIndex  = 1;
            }

         case 1 : // hold zero speed, home set
         case 0 : // hold zero speed, home not set
            setVelY( 0.0f );
            break;
      }
   }


   void coreXY_machine_type::homeAxisZ( float velocity )
   {
      float speed = C_motor.getSpeed(); 
      
      switch( Z_homeIndex )
      {
         case 6 : // fast advance
         case 4 : // slow advance
            if( digitalRead( Z_ENDSTOP_PIN ) == Z_ENDSTOP_NO_CONTACT )  // check if endstop has NOT been touched
            {
               speed += MACHINE_VEL_STEP_Z * Z_HOME_DIRECTION;
               if( speed * Z_HOME_DIRECTION > velocity ) speed = velocity * Z_HOME_DIRECTION;
               
               C_motor.setSpeed( speed );  // move toward end stop if no contact is observed
               break;
            }
            else
            {
               C_motor.setPosition( Z_HOME_OFFSET ); // switched has been activated
               Z_homeIndex--;
            }
          
         case 5 : // fast retract
         case 3 : // slow retract
            if( C_motor.getPositionMM() * Z_HOME_DIRECTION > Z_HOME_OFFSET - SLOW_HOME_DIST * Z_HOME_DIRECTION )
            {
               speed -= MACHINE_VEL_STEP_Z * Z_HOME_DIRECTION;
               if( speed * Z_HOME_DIRECTION < -velocity ) speed = -velocity * Z_HOME_DIRECTION;
               
               C_motor.setSpeed( speed );  // back away from switch
               break;
            }
            else
            {
               Z_homeIndex--;
            }
            
         case 2 : // decelerate to zero after slow retract
            speed += MACHINE_VEL_STEP_XY * Z_HOME_DIRECTION;
            
            if( speed * Z_HOME_DIRECTION < 0.0f )
            {
               C_motor.setSpeed( speed );  // decelerate
               break;
            }
            else
            {
               Z_homeIndex  = 1;
            }

         case 1 : // hold zero speed, home set
         case 0 : // hold zero speed, home not set
            C_motor.setSpeed( 0.0f );
            break;
      }
   }


   bool coreXY_machine_type::executeHome()
   {
      if(homingActive)
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


   void coreXY_machine_type::abortHome()
   {
      if( X_homeIndex > 1 || Y_homeIndex > 1 || Z_homeIndex > 1 ) // only abort if at least one axis is actively homing
      {
         if( X_homeIndex > 1 ) X_homeIndex = 0; // invalidate any axis that is currenly homing
         if( Y_homeIndex > 1 ) Y_homeIndex = 0;
         if( Z_homeIndex > 1 ) Z_homeIndex = 0;
         homingActive = true; // set to true to force a final execute that sets the motors to zero vel
         homingComplete = false;
      }
   }


   bool coreXY_machine_type::allHomeCompleted()
   {
      return homingComplete;
   }


   float coreXY_machine_type::getVelX()
   {
      return ( A_motor.getSpeed() + B_motor.getSpeed() ) * 0.5f;
   }


   float coreXY_machine_type::getVelY()
   {
      return ( A_motor.getSpeed() - B_motor.getSpeed() ) * 0.5f;
   }


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


   float coreXY_machine_type::getPosX()
   {
      return ( A_motor.getPositionMM() + B_motor.getPositionMM() ) * 0.5f;
   }


   float coreXY_machine_type::getPosY()
   {
      return ( A_motor.getPositionMM() - B_motor.getPositionMM() ) * 0.5f;
   }


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


#endif