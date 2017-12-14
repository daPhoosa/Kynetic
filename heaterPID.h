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
   
      float p_Out, i_Out, d_Out;
   
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
   float error = setTemp - probeTemp;

   p_Out = pGain * error;       // proportional component

   if( abs(p_Out) < outputMax ) // only add I+D when P is not saturated
   {
      float scaleFactor = (outputMax - abs(p_Out)) / outputMax; // soft I+D effect at extreme error
      
      iBucket = iBucket + iGain * error * scaleFactor;     // integral component 
      iBucket = constrain( iBucket, -outputMax, outputMax );
      i_Out = iBucket;        

      d_Out = dGain * ( error - lastError ) * scaleFactor; // derivative component
      d_Out = constrain( d_Out, -outputMax, outputMax );
   }
   else
   {
      iBucket = i_Out = d_Out = 0.0f;
   }

   lastError = error;
   
   output = int( p_Out + i_Out + d_Out );
   return output;
}


int  heaterPID::out()
{
   return output;
}

