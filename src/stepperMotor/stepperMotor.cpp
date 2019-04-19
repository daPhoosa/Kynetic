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

#include "stepperMotor.h"


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

   if( abs(t_feedRate) < 0.1f )
   {
      feedRate = 0.0f;
      ticksPerStep = 0;
      moveDirection = stopped;
      return;
   }

   if( t_feedRate < 0.0f )    // REVERSE
   {
      feedRate = max( -maxFeedRate, t_feedRate );  // constrain

      uint32_t tps = uint32_t( stepperConstant * -feedRate );

      noInterrupts();
      digitalWrite( directionPin, REVERSE );
      moveDirection = negative;
      ticksPerStep = tps;
      interrupts();
   }
   else                       // FORWARD
   {
      feedRate = min( maxFeedRate, t_feedRate );  // constrain

      uint32_t tps = uint32_t( stepperConstant * feedRate );

      noInterrupts();
      digitalWrite( directionPin, FORWARD );
      moveDirection = positive;
      ticksPerStep = tps;
      interrupts();
   }
}


void stepperMotor::setSpeedByPostionMM( float targetPosMM, float Hz )
{
   float newSpeed = ( targetPosMM + targetPosMM - targetPosPrev - getPositionMM() ) * Hz; //  = estimate speed + correct error
   targetPosPrev  = targetPosMM;
   setSpeed( newSpeed );
}


void stepperMotor::setTickRateHz( const uint32_t & t_tickRateHz )
{
   tickRateHz = float(t_tickRateHz);
   maxFeedRate = 0.5f * tickRateHz * MMPerStep; // max feed rate is when sending a pulse every other tick
   stepperConstant = float(1UL << 31) / maxFeedRate;
}


void stepperMotor::setPosition( float posFloat )
{
   targetPosPrev = posFloat;

   posFloat *= stepsPerMM; // convert to steps

   int32_t posInt  = round(posFloat); // whole steps ( use round to deal with decimal truncation)
   int32_t posFrac = (posFloat - float(posInt)) * MAX_UINT_32; // fractional step

   noInterrupts();
   position    = posInt;
   tickCounter = posFrac;
   interrupts();
}


float stepperMotor::getPositionMM()
{
   return (float(position) + float(tickCounter) * (1.0f / MAX_UINT_32) ) * MMPerStep;
}


float stepperMotor::getSpeed()
{
   return feedRate;  // MM / s
}



