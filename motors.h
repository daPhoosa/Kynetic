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
#include "stepperMotor.h"

volatile uint32_t stepperTickCount = STEPPER_TICK_HZ;  // used to track actual tick rate

// **** MOTOR SETUP ****
stepperMotor A_motor( A_MOTOR_STEP_PER_MM, A_MOTOR_DIRECTION, STEPPER_TICK_HZ, A_MOTOR_STEP_PIN, A_MOTOR_DIR_PIN );
stepperMotor B_motor( B_MOTOR_STEP_PER_MM, B_MOTOR_DIRECTION, STEPPER_TICK_HZ, B_MOTOR_STEP_PIN, B_MOTOR_DIR_PIN );
stepperMotor C_motor( C_MOTOR_STEP_PER_MM, C_MOTOR_DIRECTION, STEPPER_TICK_HZ, C_MOTOR_STEP_PIN, C_MOTOR_DIR_PIN );
stepperMotor D_motor( D_MOTOR_STEP_PER_MM, D_MOTOR_DIRECTION, STEPPER_TICK_HZ, D_MOTOR_STEP_PIN, D_MOTOR_DIR_PIN );


void armMotors()
{
   /*
   A_motor.enable();
   B_motor.enable();
   C_motor.enable();
   D_motor.enable();
   */

   pinMode( A_MOTOR_ENBL_PIN, LOW);
   pinMode( B_MOTOR_ENBL_PIN, LOW);
   pinMode( C_MOTOR_ENBL_PIN, LOW);
   pinMode( D_MOTOR_ENBL_PIN, LOW);

}


void MotorControlISR() // at 60mm/s with 100k tick rate: 9.6% CPU usage
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

            if( abs(deltaA) > 1.0f / A_MOTOR_STEP_PER_MM )
            {
               A_motor.setSpeed( float(MOTION_CONTROL_HZ) * deltaA );
            }
            else
            {
               A_motor.setSpeed( 0 );
            }
            break;

         case 3:
            deltaB = motor_B - B_motor.getPositionMM();

            if( abs(deltaB) > 1.0f / B_MOTOR_STEP_PER_MM )
            {
               B_motor.setSpeed( float(MOTION_CONTROL_HZ) * deltaB );
            }
            else
            {
               B_motor.setSpeed( 0 );
            } 
            break;

         case 4:
            deltaC = motor_C - C_motor.getPositionMM();

            if( abs(deltaC) > 1.0f / C_MOTOR_STEP_PER_MM )
            {
               C_motor.setSpeed( float(MOTION_CONTROL_HZ) * deltaC );
            }
            else
            {
               C_motor.setSpeed( 0 );
            }
            break;

         case 5:
            extrudeDelta = motion.getExtrudeLocationMM() - D_motor.getPositionMM();

            if( abs(extrudeDelta) > 1.0f / D_MOTOR_STEP_PER_MM ) // error must be more than 1 step, or motor is stopped
            {  
               D_motor.setSpeed( float(MOTION_CONTROL_HZ) * extrudeDelta );
            }
            else
            {
               D_motor.setSpeed( 0 );
            }
            break;

         default:
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

void setMotorTickRate()
{
   // update tick rate to account for unexpected ISR call rates at high Hz
   // this might not be needed, but some frequencies are not available, so this will mitigate the error

   static uint32_t startTime;
   uint32_t timeNow = micros();
   uint32_t elapsedTime = timeNow - startTime;
   startTime = timeNow;

   if( elapsedTime < 1100000UL && elapsedTime > 900000UL ) // don't update if excessively delayed/early
   {
      float scaleFactor = 1000000.0f / float(elapsedTime);
      uint32_t tickCount = scaleFactor * float(stepperTickCount) + 0.5f;

      A_motor.setTickRateHz( tickCount );
      B_motor.setTickRateHz( tickCount );
      C_motor.setTickRateHz( tickCount );
      D_motor.setTickRateHz( tickCount );

      KORE.motionTickPerExecute = uint32_t( float(tickCount) / float(MOTION_CONTROL_HZ) + 0.5f );

      //SERIAL_PORT.println( tickCount );
      //SERIAL_PORT.println( scaleFactor, 6 );
   }
   stepperTickCount = 0;
}
