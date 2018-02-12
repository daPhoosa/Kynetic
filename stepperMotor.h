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

         void setStepperConstant();
         void setMaxFeedRate();

         float stepperConstant;
         float tickRateHz;
         float stepsPerMM, MMPerStep;

         float feedRate, maxFeedRate;

         int directionPin, stepPin;
         int FORWARD, REVERSE;
         bool stepPinOn;

         volatile uint32_t ticksPerStep, tickCounter;

         volatile int32_t position;

         enum move_direction_t {
            Positive,
            Negative,
            Stopped
         } moveDirection;

   };


   stepperMotor::stepperMotor( float t_stepsPerMM, int t_direction, uint32_t t_tickRateHz, int t_stepPin, int t_dirPin )
   {
      stepsPerMM = t_stepsPerMM;
      MMPerStep  = 1.0f / stepsPerMM;
      setTickRateHz( t_tickRateHz );

      moveDirection = Stopped;
      ticksPerStep = 1 << 31;

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
      feedRate = constrain( t_feedRate, -maxFeedRate, maxFeedRate );

      static const float minFeedRate = 0.1f;

      if( feedRate < -minFeedRate )     // reverse
      {
         uint32_t tps = uint32_t( stepperConstant / -feedRate + 0.5f );
         noInterrupts();
         digitalWrite(directionPin, REVERSE);
         moveDirection = Negative;
         ticksPerStep = tps;
         interrupts();
      }
      else if( feedRate > minFeedRate ) // forward
      {
         uint32_t tps = uint32_t( stepperConstant / feedRate + 0.5f );
         noInterrupts();
         digitalWrite(directionPin, FORWARD);
         moveDirection = Positive;
         ticksPerStep = tps;
         interrupts();
      }
      else                              // stopped
      {
         noInterrupts();
         moveDirection = Stopped;
         ticksPerStep = 1 << 31;
         tickCounter = 0;
         interrupts();
      }
   }


   void stepperMotor::setTickRateHz( const uint32_t & t_tickRateHz )
   {
      tickRateHz = float(t_tickRateHz);
      setStepperConstant();
      setMaxFeedRate();
   }


   void stepperMotor::setPosition(const float & posFloat)
   {
      int32_t posInt = int32_t( posFloat * stepsPerMM + 0.5f );
      
      noInterrupts();
      position = posInt;
      interrupts();
   }


   void stepperMotor::setStepperConstant()
   {
      stepperConstant = tickRateHz * MMPerStep;
   }


   void stepperMotor::setMaxFeedRate()
   {
      maxFeedRate = 0.5f * tickRateHz * MMPerStep; // max feed rate is when sending a pulse every other tick
   }

   float stepperMotor::getPositionMM()
   {
      noInterrupts();
      int32_t temp = position;
      interrupts();
      
      return float(temp) * MMPerStep;
   }

   inline void stepperMotor::step()
   {
      tickCounter++;

      if( tickCounter >= ticksPerStep )
      {
         if( stepPinOn )
         {
            digitalWrite( stepPin, LOW );   // set pin low
            stepPinOn = false;
            tickCounter = 1;
         }
         else
         {
            if( moveDirection == Positive )
            {
               position++;
               digitalWrite( stepPin, HIGH );
               stepPinOn = true;
            }
            else if( moveDirection == Negative )
            {
               position--;
               digitalWrite( stepPin, HIGH );
               stepPinOn = true;
            }
            else     // stopped
            {
               tickCounter = 0; // reset to avoid running repeatedly
               // no step pulse is sent if stopped
            }  
         }
      }      
   }

#endif