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

#include <FrequencyTimer2.h>     // https://github.com/PaulStoffregen/FrequencyTimer2     --included with teensyduino
#include <SmoothMove.h>          // https://github.com/daPhoosa/SmoothMove                --instal to libraries diectory


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
   //uint32_t timeNow = micros();

   static uint32_t counter = 0;
   static float  cart_X,  cart_Y,  cart_Z;
   static float motor_A, motor_B, motor_C;
   static float  deltaA,  deltaB,  deltaC, extrudeDelta;

   if( KORE.runProgram && machine.allHomeCompleted() )
   {
      switch( counter ) // split motion control over multiple ISR calls
      {
         case 0:
            motion.getTargetLocation( cart_X, cart_Y, cart_Z );
            break;

         case 1:
            machine.invKinematics( cart_X, cart_Y, cart_Z, motor_A, motor_B, motor_C );
            break;

         case 2:
            deltaA = motor_A - A_motor.getPositionMM();
            deltaB = motor_B - B_motor.getPositionMM();
            deltaC = motor_C - C_motor.getPositionMM();

            if( abs(deltaA) > 1.0f / A_MOTOR_STEP_PER_MM ){
               A_motor.setSpeed( float(MOTION_CONTROL_HZ) * deltaA );
            }else{
               A_motor.setSpeed( 0 );
            }
            break;

            if( abs(deltaB) > 1.0f / B_MOTOR_STEP_PER_MM ){
               B_motor.setSpeed( float(MOTION_CONTROL_HZ) * deltaB );
            }else{
               B_motor.setSpeed( 0 );
            } 
            break;

            if( abs(deltaC) > 1.0f / C_MOTOR_STEP_PER_MM ){
               C_motor.setSpeed( float(MOTION_CONTROL_HZ) * deltaC );
            }else{
               C_motor.setSpeed( 0 );
            }
            break;

         case 3:
            extrudeDelta = motion.getExtrudeLocationMM() - D_motor.getPositionMM();

            if( abs(extrudeDelta) > 1.0f / D_MOTOR_STEP_PER_MM ){  
               D_motor.setSpeed( float(MOTION_CONTROL_HZ) * extrudeDelta );
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

   //funCounter += micros() - timeNow;
}

void startStepperTickISR()
{
   pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
   FrequencyTimer2::setPeriod(STEPPER_TICK_PERIOD);
   FrequencyTimer2::setOnOverflow(MotorControlISR);
}