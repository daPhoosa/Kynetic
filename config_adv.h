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


// ***********************************
// **** CONTROL LOOP FREQUENCIES ****
// *********************************
const int MOTION_CONTROL_HZ     = 2000;    // Hz
const int BLOCK_EXECUTE_HZ      = 400;     // Hz
const int SOFT_PWM_HZ           = 100;     // Hz
const int HEATER_MANAGER_HZ     = 10;      // Hz
const int BUTTONS_UI_HZ         = 50;      // Hz
const int MAINTENANCE_HZ        = 1;       // Hz

const float MACHINE_VEL_STEP = MACHINE_ACCEL / float(MOTION_CONTROL_HZ); // [mm/s*step]  max vel change per step at max acceleration


// ***********************************
// **** STEPPER TICK FREQUENCIES ****
// *********************************
const int   STEPPER_TICK_PERIOD = 10;      // us
const uint32_t STEPPER_TICK_HZ  = 1000000UL / STEPPER_TICK_PERIOD; // Hz




struct kynetic_operation_retention_enabler_t
{
   int extrude1TargetTemp = 0;
   bool extrude1_wait = false;

   int bedTargetTemp = 0;
   bool bed_wait = false;
   
} KORE;