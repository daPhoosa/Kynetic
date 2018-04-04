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

   if( !machine.homingActive() )
   {
      static uint32_t counter = 10;

      static float  X,  Y,  Z;      // cartesian coordinates
      static float  A,  B,  C;      // motor positions

      switch( counter ) // split motion control over multiple ISR calls to avoid going over time
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

         case 3:                                       // set motion motor speeds
            A_motor.setSpeed( float(MOTION_CONTROL_HZ) * (A - A_motor.getPositionMM()) );
            B_motor.setSpeed( float(MOTION_CONTROL_HZ) * (B - B_motor.getPositionMM()) );
            C_motor.setSpeed( float(MOTION_CONTROL_HZ) * (C - C_motor.getPositionMM()) );
            break;

         case 4:                                       // set extruder motor speed
            D_motor.setSpeed( float(MOTION_CONTROL_HZ >> 1) * (motion.getExtrudeLocationMM() - D_motor.getPositionMM()) );
            break;

         default:
            break;
      }
      counter++;

      static uint32_t bucket = 0; // use integer rollover to time motion control
      uint32_t prev = bucket;
      bucket += KORE.motionTickPerExecute;
      if( bucket < prev ) counter = 0; // reset on rollover

   }

   A_motor.step();
   B_motor.step();
   C_motor.step();
   D_motor.step();

   stepperTickCount++;

   //funCounter += micros() - timeNow;
}

void startStepperTickISR()
{
   pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
   FrequencyTimer2::setPeriod(STEPPER_TICK_PERIOD);
   FrequencyTimer2::setOnOverflow(MotorControlISR);
}