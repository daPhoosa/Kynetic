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


class heaterPID
{
   public:
   
      heaterPID( int Hz, float p, float i, float d );
      
      void setGain( float p, float i, float d );
      
      int in( float setTemp, float probeTemp );
      int out();
   
   
   private:
   
      float pGain, iGain, dGain;
      
      float iBucket;
      
      float sampleRateHz;
      
      float lastError, setPointTemp;

      const float outputMax = 255.0f;
      const float outputMin =   0.0f;
      
      int output;

};


heaterPID::heaterPID( int Hz, float p, float i, float d )
{
   sampleRateHz = Hz;
   setGain( p, i, d );
}


void heaterPID::setGain( float p, float i, float d )
{
   pGain = p;
   iGain = i / sampleRateHz;
   dGain = d * sampleRateHz;
}


int heaterPID::in( float setTemp, float probeTemp )
{
   float result;
   float error = setTemp - probeTemp;
   
   result = pGain * error;                   // proportional component

   if( abs(error) < 10.0f )
   {
      result -= dGain * ( lastError - error );   // derivative component
      result  = constrain( result, outputMin, outputMax );
      
      iBucket += iGain * error;
      //iBucket  = constrain( iBucket, -result, outputMax - result );  // prevent bucket from accumulating past saturation point
      iBucket  = constrain( iBucket, -outputMax, outputMax );
      result  += iBucket;
   }
   else
   {
      iBucket = 0.0f;
   }


   lastError = error;

   output = int( result );
   return output;
}


int  heaterPID::out()
{
   return output;
}

