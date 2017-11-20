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

/*
      Expected Input range 0-255 to match Arduino AnalogWrite function

      Off and on periods will never be less than minPeriod
*/


class slowPWM
{
   public:

      slowPWM( int periodMS );

      void set( int rate );
      bool check();


   private:

      uint32_t onTime;
      uint32_t offTime;

      uint32_t onPeriod;
      uint32_t offPeriod;

      uint32_t minPeriod; // [ms]

      bool onNow;

};


void slowPWM::slowPWM( int periodMS )
{
   minPeriod = abs(periodMS);
}


void slowPWM::set(int rate)
{

   if( rate < 1 )  // full off
   {
      onPeriod  = 0;
      offPeriod = 4.29e9;
      return;
   }

   if( rate > 254 )  // full on
   {
      onPeriod  = 4.29e9;
      offPeriod = 0;
      return;
   }

   if( rate < 128 )  // less than 50%
   {
      onPeriod  = minPeriod;
      offPeriod = ( minPeriod << 8 ) / rate - minPeriod;
   }
   else              // over 50%
   {
      offPeriod = minPeriod;
      onPeriod  = ( minPeriod << 8 ) / (255 - rate) - minPeriod;
   }

}


bool slowPWM::check()
{
   uint32_t timeNow = millis();

   if( onNow )
   {
      // currently on, check for turn off
      if( timeNow - onTime > onPeriod )
      {
         onNow   = false;
         offTime = timeNow;
         return false;
      }
   }
   else
   {
      // currently off, check for turn on
      if( timeNow - offTime > offPeriod )
      {
         onNow  = true;
         onTime = timeNow;
         return true;
      }
   }
}



