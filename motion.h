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


// **** MOVEMENT ENGINE ****
SmoothMove motion;


void resetPosition( const float & x, const float & y, const float & z )
{
   noInterrupts();
   motion.abortMotion();
   motion.setPosition( x, y, z );
   gCodeSetPosition(   x, y, z );

   float a, b, c;
   machine.invKinematics( x, y, z, a, b, c ); // convert new position to motor coordinates
   A_motor.setPosition( a );  // set motor positions
   B_motor.setPosition( b );
   C_motor.setPosition( c );

   motion.startMoving();
   interrupts();
}


void configMotion()
{
   motion.setParamXY( MACHINE_ACCEL_XY, MAX_VELOCITY_XY );
   motion.setParamZ( MACHINE_ACCEL_Z, MAX_VELOCITY_Z );

   motion.setCornerRounding( CORNER_ROUNDING );
   motion.setJunctionVelRad( JUNCTION_VEL_RAD );

   motion.setExtrudeRateOverride( 1.0f );
   motion.setMotionRateOverride(  1.0f );
   motion.setExrudeAccel( EXTRUDE_ACCEL );
   motion.setLookAheadTime( 150 );
   motion.setExtrudeVelocityAdvance( VEL_EXTRUDE_ADV );
   motion.junctionSmoothingOff();

   resetPosition( 0.0f, 0.0f, 0.0f );
}


void MotorControlISR() // at 60mm/s with 100k tick rate: xxxx CPU usage
{
   //uint32_t timeNow = micros();

   // GENERATE MOTOR STEPS (execute first to reduce jitter)
   A_motor.step();
   B_motor.step();
   C_motor.step();
   D_motor.step();
   stepperTickCount++;

   static bool runMotion = false;
   static uint32_t bucket = 0; // use integer rollover to time motion control
   uint32_t prev = bucket;
   bucket += KORE.motionTickPerExecute;
   if( bucket < prev ) runMotion = true; // reset on rollover

   if( runMotion )
   {
      if( machine.homingActive() )  // *** HOMING
      {
         if( machine.executeHome() )  // Home operation
         {
            Vec3 cart;

            machine.fwdKinematics( A_motor.getPositionMM(), B_motor.getPositionMM(), C_motor.getPositionMM(), cart.x, cart.y, cart.z ); // compute current cartesian start location

            motion.setPosition( cart.x, cart.y, cart.z );
            gCodeSetPosition(   cart.x, cart.y, cart.z ); 

            startPollTimers();

            KORE.runProgram = true;
            KORE.heaterWatchDog = 0;

            //motionControl.resetStats();
            //blockRead.resetStats();
            
            motion.startMoving();

            display("Home Complete \n");
         }
         runMotion = false;
      }
      else                          // *** NORMAL MOTION
      {
         static int counter = 0;

         static float  X,  Y,  Z;      // cartesian coordinates
         static float  A,  B,  C;      // motor positions

         float position, speed;

         switch( counter++ ) // split motion control over multiple ISR calls to avoid going over time
         {
            case 0:
               motion.advancePostion();
               break;

            case 1:
               motion.getTargetLocation( X, Y, Z );  // get next position in cartesian space
               break;

            case 2:
               machine.invKinematics( X, Y, Z, A, B, C ); // convert position to motor coordinates
               break;

            case 3:                                       // set extruder motor speed
               speed    = D_motor.getSpeed();
               position = motion.getExtrudeLocationMM() + speed * VEL_EXTRUDE_ADV;   // add velocity advance
               D_motor.setSpeedByPostionMM( position, float(MOTION_CONTROL_HZ) );
               break;

            case 4:                                       // set motion motor speeds
               A_motor.setSpeedByPostionMM( A, float(MOTION_CONTROL_HZ) );
               B_motor.setSpeedByPostionMM( B, float(MOTION_CONTROL_HZ) );
               C_motor.setSpeedByPostionMM( C, float(MOTION_CONTROL_HZ) );
               runMotion = false;
               counter   = 0;
               break;

            default:
               break;
         }
      }
   }

   //funCounter += micros() - timeNow;
}

void startStepperTickISR()
{
   pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
   FrequencyTimer2::setPeriod(STEPPER_TICK_PERIOD);
   FrequencyTimer2::setOnOverflow(MotorControlISR);
}