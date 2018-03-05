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

void configMotion()
{
   motion.setParamXY( MACHINE_ACCEL_XY, MAX_VELOCITY_XY );
   motion.setParamZ( MACHINE_ACCEL_Z, MAX_VELOCITY_Z );
   motion.setCornerRounding( CORNER_ROUNDING );
         
   motion.setExtrudeRateOverride( 1.0f );
   motion.setMotionRateOverride(  1.0f );
   motion.setExrudeAccel( EXTRUDE_ACCEL );
   //motion.junctionSmoothingOff();
}


void MotorControlISR() // at 60mm/s with 100k tick rate: xxxx CPU usage
{
   uint32_t timeNow = micros();

   static uint32_t counter = 0;
   static float  cart_X,  cart_Y,  cart_Z;
   static float motor_A, motor_B, motor_C;
   static float  deltaA,  deltaB,  deltaC, extrudeDelta;

   if( KORE.runProgram && machine.allHomeCompleted() )
   {
      switch( counter ) // split motion control over multiple ISR calls to avoid going over time
      {
         case 0:
            motion.getTargetLocation( cart_X, cart_Y, cart_Z );  // get next position in cartesian space
            break;

         case 1:
            machine.invKinematics( cart_X, cart_Y, cart_Z, motor_A, motor_B, motor_C ); // convert position to motor coordinates 
            break;

         case 2:                                            // set motion motor speeds
            deltaA = motor_A - A_motor.getPositionMM();
            deltaB = motor_B - B_motor.getPositionMM();
            deltaC = motor_C - C_motor.getPositionMM();
            
            if( abs(deltaA) > 0.5f / A_MOTOR_STEP_PER_MM ){
               A_motor.setSpeed( float(MOTION_CONTROL_HZ >> 1) * deltaA );
            }else{
               A_motor.setSpeed( 0 );
            }

            if( abs(deltaB) > 0.5f / B_MOTOR_STEP_PER_MM ){
               B_motor.setSpeed( float(MOTION_CONTROL_HZ >> 1) * deltaB );
            }else{
               B_motor.setSpeed( 0 );
            } 

            if( abs(deltaC) > 0.5f / C_MOTOR_STEP_PER_MM ){
               C_motor.setSpeed( float(MOTION_CONTROL_HZ >> 1) * deltaC );
            }else{
               C_motor.setSpeed( 0 );
            }
            break;

         case 3:                                            // set extruder speed
            extrudeDelta = motion.getExtrudeLocationMM() - D_motor.getPositionMM();

            if( abs(extrudeDelta) > 0.5f / D_MOTOR_STEP_PER_MM ){  
               D_motor.setSpeed( float(MOTION_CONTROL_HZ >> 1) * extrudeDelta );
            }else{
               D_motor.setSpeed( 0 );
            }
            break;

         default:
            break;
      }

      if( counter >= KORE.motionTickPerExecute )
      {
         counter = 0;
      }
      else
      {
         counter++;
      }
   }
   else
   {
      counter = 0;
   }

   A_motor.step();
   B_motor.step();
   C_motor.step();
   D_motor.step();
   
   stepperTickCount++;

   funCounter += micros() - timeNow;
}

void startStepperTickISR()
{
   pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
   FrequencyTimer2::setPeriod(STEPPER_TICK_PERIOD);
   FrequencyTimer2::setOnOverflow(MotorControlISR);
}