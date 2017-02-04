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
#include <uStepper.h>            // https://github.com/daPhoosa/uStepper                  --instal to libraries diectory

volatile uint32_t stepperTickCount = STEPPER_TICK_HZ;  // used to track actual tick rate

// **** MOTOR SETUP ****
uStepper A_motor( A_MOTOR_STEP_PER_MM,  A_MOTOR_DIRECTION,  STEPPER_TICK_HZ, A_MOTOR_STEP_PIN,  A_MOTOR_DIR_PIN,  A_MOTOR_ENBL_PIN);
uStepper B_motor( B_MOTOR_STEP_PER_MM,  B_MOTOR_DIRECTION,  STEPPER_TICK_HZ, B_MOTOR_STEP_PIN,  B_MOTOR_DIR_PIN,  B_MOTOR_ENBL_PIN);
uStepper C_motor( C_MOTOR_STEP_PER_MM,  C_MOTOR_DIRECTION,  STEPPER_TICK_HZ, C_MOTOR_STEP_PIN,  C_MOTOR_DIR_PIN,  C_MOTOR_ENBL_PIN);
uStepper D_motor(D_MOTOR_STEP_PER_MM, D_MOTOR_DIRECTION, STEPPER_TICK_HZ, D_MOTOR_STEP_PIN, D_MOTOR_DIR_PIN, D_MOTOR_ENBL_PIN);


void MotorTickISR() 
{
   A_motor.step();
   B_motor.step();
   C_motor.step();
   D_motor.step();
   
   stepperTickCount++;
}

void startStepperTickISR()
{
   pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
   FrequencyTimer2::setPeriod(STEPPER_TICK_PERIOD);
   FrequencyTimer2::setOnOverflow(MotorTickISR);
}