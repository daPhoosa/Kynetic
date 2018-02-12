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
         void setPosition( const int32_t & posInt );

         float getPositionMM();
         int32_t getPositionSteps();
         float getSpeed();

         inline void step();


      private:

         void setStepperConstant();
         void setMaxFeedRate();

         float stepperConstant;
         float tickRateHz;
         float stepsPerMM;

         float feedRate, maxFeedRate, minFeedRate;

         int directionPin, stepPin;
         int FORWARD, REVERSE;

         volatile uint32_t ticksPerStep;


         enum moveDir_t {
            Positive,
            Negative,
            Stopped
         } moveDirection;

   }


   stepperMotor::stepperMotor( float t_stepsPerMM, int t_direction, uint32_t t_tickRateHz, int t_stepPin, int t_dirPin )
   {
      stepsPerMM = t_stepsPerMM;
      tickRateHz = float(t_tickRateHz);
      setStepperConstant();
      setMaxFeedRate();

   }


   void stepperMotor::setSpeed( float t_feedRate )
   {
      feedrate = constrain( t_feedRate, -maxFeedRate, maxFeedRate );

      static const float minFeedRate = 0.1f;

      if( feedRate < -minFeedRate ) // reverse
      {

      }
      else if ( feedRate > minFeedRate ) // forward
      {

      }
      else // stopped
      {

      }


   }


   void stepperMotor::setStepperConstant()
   {
      stepperConstant = tickRateHz / stepsPerMM;
   }


   void setMaxFeedRate()
   {
      maxFeedRate = tickRateHz / ( 2.0f * stepsPerMM ); // max fed rate is when sending a pulse every other tick
   }


#endif