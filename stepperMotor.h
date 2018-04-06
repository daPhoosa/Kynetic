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
         void setTickRateHz( const uint32_t & t_tickRateHz );
         void setPosition( const float & posFloat );

         float getPositionMM();
         float getSpeed();

         inline void step();


      private:

         float stepperConstant;
         float tickRateHz;
         float stepsPerMM, MMPerStep;

         float feedRate, maxFeedRate;

         int directionPin, stepPin;
         int FORWARD, REVERSE;

         volatile bool stepPinOn;
         volatile uint32_t ticksPerStep, tickCounter;
         volatile int32_t position;

         bool movePos;

   };


   stepperMotor::stepperMotor( float t_stepsPerMM, int t_direction, uint32_t t_tickRateHz, int t_stepPin, int t_dirPin )
   {
      stepsPerMM = t_stepsPerMM;
      MMPerStep  = 1.0f / stepsPerMM;
      setTickRateHz( t_tickRateHz );

      ticksPerStep = 0;

      // Config Step Pin
      stepPin = t_stepPin;
      pinMode(stepPin, OUTPUT);
      digitalWrite(stepPin, LOW);
      stepPinOn = false;

      // Config Direction Pin
      directionPin = t_dirPin;
      pinMode(directionPin, OUTPUT);
      digitalWrite(directionPin, LOW);

      if( t_direction > 0 )
      {
         FORWARD = 1;
         REVERSE = 0;
      }
      else
      {
         FORWARD = 0;
         REVERSE = 1;
      }

   }


   void stepperMotor::setSpeed( float t_feedRate )
   {
      if( abs(t_feedRate) < 1.0f )
      {
         feedRate = 0.0f;
         ticksPerStep = 0;
         return;
      }

      if( t_feedRate < 0.0f )    // REVERSE
      {
         feedRate = max( -maxFeedRate, t_feedRate );  // constrain

         uint32_t tps = uint32_t( stepperConstant * -feedRate );

         noInterrupts();
         digitalWrite( directionPin, REVERSE );
         movePos = false;
         ticksPerStep = tps;
         interrupts();
      }
      else                       // FORWARD
      {
         feedRate = min( maxFeedRate, t_feedRate );  // constrain

         uint32_t tps = uint32_t( stepperConstant * feedRate );

         noInterrupts();
         digitalWrite( directionPin, FORWARD );
         movePos = true;
         ticksPerStep = tps;
         interrupts();
      }
   }


   void stepperMotor::setTickRateHz( const uint32_t & t_tickRateHz )
   {
      tickRateHz = float(t_tickRateHz);
      maxFeedRate = 0.5f * tickRateHz * MMPerStep; // max feed rate is when sending a pulse every other tick
      stepperConstant = float(1UL << 31) / maxFeedRate;
   }


   void stepperMotor::setPosition(const float & posFloat)
   {
      if( posFloat > 0.0f )
      {
         position = int32_t( posFloat * stepsPerMM + 0.5f );
      }
      else
      {
         position = int32_t( posFloat * stepsPerMM - 0.5f );
      }
   }


   float stepperMotor::getPositionMM()
   {
      return (float(position) + float(tickCounter) * (1.0f / 4294967296.0f) ) * MMPerStep;
   }


   float stepperMotor::getSpeed()
   {
      return feedRate;  // MM / s
   }


   inline void stepperMotor::step()
   {
      uint32_t prev = tickCounter;

      if( stepPinOn )
      {
         digitalWrite( stepPin, LOW );   // set pin low
         stepPinOn = false;
      }

      if( movePos )  // POSITIVE
      {
         tickCounter += ticksPerStep;
         if( tickCounter < prev )
         {
            digitalWrite( stepPin, HIGH );
            stepPinOn = true;
            position++;
         }
      }
      else           // NEGATIVE
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