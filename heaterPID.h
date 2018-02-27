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
   
      heaterPID( int Hz, float p, float i, float d, float f );
      
      void setGain( float p, float i, float d, float f );
      void setAmbTemp( float t );
      
      int in( float setTemp, float probeTemp );
      int out();

      int getSaturationTime();

      void  display();
   

   private:

      float setTemp, probeTemp, ambTemp;
   
      float pGain, iGain, dGain, fwdGain;
      
      float p_Out, i_Out, d_Out, fwd_Out;
      
      float sampleRateHz;
      
      float lastError, setPointTemp;

      const float OUTPUT_MAX = 255.0f;
      const float INV_OUTPUT_MAX = 1.0f / OUTPUT_MAX;
      
      int output;

      bool outputSaturated;
      uint32_t saturationStartTime;
};


heaterPID::heaterPID( int Hz, float p, float i, float d, float f )
{
   sampleRateHz = Hz;
   setGain( p, i, d, f );
   setAmbTemp( 22.0f );
}


void heaterPID::setGain( float p, float i, float d, float f )
{
   pGain = p;
   iGain = i / sampleRateHz;
   dGain = d * sampleRateHz;
   fwdGain = f;
}


void heaterPID::setAmbTemp( float t )
{
   ambTemp = t;
}


int heaterPID::in( float set, float probe )
{
   setTemp = set;
   probeTemp = probe;

   fwd_Out = fwdGain * ( setTemp - ambTemp );

   float error = setTemp - probeTemp;
   float errorDelta = (error - lastError) * 0.25f; // smooth error change

   p_Out = constrain( pGain * error, -OUTPUT_MAX, OUTPUT_MAX );   // proportional component

   if( abs(p_Out) < OUTPUT_MAX ) // only add I+D when P is not saturated
   {
      float scaleFactor = (OUTPUT_MAX - abs(p_Out)) * INV_OUTPUT_MAX; // soften I+D effect at extreme error

      i_Out += iGain * error * scaleFactor;     // integral component 
      i_Out = constrain( i_Out, -OUTPUT_MAX, OUTPUT_MAX );     

      d_Out = dGain * errorDelta * scaleFactor; // derivative component
      d_Out = constrain( d_Out, -OUTPUT_MAX, OUTPUT_MAX );
   }
   else
   {
      i_Out = d_Out = 0.0f;
   }

   lastError += errorDelta; 
   
   output = int( p_Out + i_Out + d_Out + fwd_Out );

   if( output >= int(OUTPUT_MAX) )
   {
      outputSaturated = true;
      if( !saturationStartTime ) saturationStartTime = millis(); // set start time the first time saturation is observed
   }
   else
   {
      outputSaturated = false;
      saturationStartTime  = 0;
   }
   
   return output;
}


int  heaterPID::out()
{
   return output;
}


int heaterPID::getSaturationTime()
{
   if( outputSaturated )
   {
      return ( millis() - saturationStartTime ) / 1000; // return time in seconds
   }

   return 0;   
}


void  heaterPID::display()
{
   if(SERIAL_PORT)
   {
      String msg = String( setTemp )   + " " + 
                   String( probeTemp ) + " " + 
                   String( p_Out )     + " " + 
                   String( i_Out )     + " " + 
                   String( d_Out )     + " " + 
                   String( fwd_Out )   + " " + 
                   String( output )    + '\n';

      SERIAL_PORT.print(msg);
   }
}
