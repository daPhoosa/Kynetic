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

volatile uint32_t stepperTickCount = STEPPER_TICK_HZ;  // used to track actual tick rate

// **** MOTOR SETUP ****
stepperMotor A_motor( A_MOTOR_STEP_PER_MM, A_MOTOR_DIRECTION, STEPPER_TICK_HZ, A_MOTOR_STEP_PIN, A_MOTOR_DIR_PIN );
stepperMotor B_motor( B_MOTOR_STEP_PER_MM, B_MOTOR_DIRECTION, STEPPER_TICK_HZ, B_MOTOR_STEP_PIN, B_MOTOR_DIR_PIN );
stepperMotor C_motor( C_MOTOR_STEP_PER_MM, C_MOTOR_DIRECTION, STEPPER_TICK_HZ, C_MOTOR_STEP_PIN, C_MOTOR_DIR_PIN );
stepperMotor D_motor( D_MOTOR_STEP_PER_MM, D_MOTOR_DIRECTION, STEPPER_TICK_HZ, D_MOTOR_STEP_PIN, D_MOTOR_DIR_PIN );


void armMotors()
{
   A_motor.setSpeed( 0 );
   B_motor.setSpeed( 0 );
   C_motor.setSpeed( 0 );
   D_motor.setSpeed( 0 );

   digitalWrite( A_MOTOR_ENBL_PIN, LOW );
   digitalWrite( B_MOTOR_ENBL_PIN, LOW );
   digitalWrite( C_MOTOR_ENBL_PIN, LOW );
   digitalWrite( D_MOTOR_ENBL_PIN, LOW );
}


void stopMotors()
{
   A_motor.setSpeed( 0 );
   B_motor.setSpeed( 0 );
   C_motor.setSpeed( 0 );
   D_motor.setSpeed( 0 );

   digitalWrite( A_MOTOR_ENBL_PIN, HIGH);
   digitalWrite( B_MOTOR_ENBL_PIN, HIGH);
   digitalWrite( C_MOTOR_ENBL_PIN, HIGH);
   digitalWrite( D_MOTOR_ENBL_PIN, HIGH);
}


void setMotorTickRate()
{
   // update tick rate to account for variation in expected ISR call rates at high Hz
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

      KORE.motionTickPerExecute = uint32_t( powf(2.0f, 32.0f) * float(MOTION_CONTROL_HZ) / float(tickCount) + 0.5f ); // update incrementer for motion control loop
   }
   stepperTickCount = 0;
}
