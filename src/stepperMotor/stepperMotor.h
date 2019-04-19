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


#ifndef stepperMotor_h
   #define stepperMotor_h

   #include <arduino.h>

   class stepperMotor
   {
      public:

         stepperMotor( float t_stepsPerMM, int t_direction, uint32_t t_tickRateHz, int t_stepPin, int t_dirPin );

         void setSpeed( float feedRate );
         void setSpeedByPostionMM( float targetPosMM, float Hz );

         void setTickRateHz( const uint32_t & t_tickRateHz );
         void setPosition( float posFloat );

         float getPositionMM();
         float getSpeed();

         inline void step();


      private:

         float stepperConstant;
         float tickRateHz;
         float stepsPerMM, MMPerStep;
         float targetPosPrev;

         float maxFeedRate;
         volatile float feedRate;

         int directionPin, stepPin;
         int FORWARD, REVERSE;

         volatile bool stepPinOn;
         volatile uint32_t ticksPerStep;
         volatile int32_t position, tickCounter;

         const static float MAX_UINT_32 = powf( 2.0f, 32.0f ) - 1.0f;

         enum move_direction_t
         {
            positive,
            negative,
            stopped
         } moveDirection;

   };


   inline void stepperMotor::step()
   {
      int32_t prev = tickCounter;

      if( stepPinOn )
      {
         digitalWrite( stepPin, LOW );   // set pin low
         stepPinOn = false;
      }

      if( moveDirection == positive )  // POSITIVE
      {
         tickCounter += ticksPerStep;
         if( tickCounter < prev )
         {
            digitalWrite( stepPin, HIGH );
            stepPinOn = true;
            position++;
         }
      }
      else if( moveDirection == negative )// NEGATIVE
      {
         tickCounter -= ticksPerStep;
         if( tickCounter > prev )
         {
            digitalWrite( stepPin, HIGH );
            stepPinOn = true;
            position--;
         }
      }
   }   

#endif