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

      void  display();
   

   private:

      float setTemp, probeTemp, ambTemp;
   
      float pGain, iGain, dGain, fwdGain;
      
      float p_Out, i_Out, d_Out, fwd_Out;
      
      float sampleRateHz;
      
      float lastError, setPointTemp;

      const float outputMax = 255.0f;
      const float invOutputMax = 1.0f / outputMax;
      
      int output;
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

   p_Out = constrain( pGain * error, -outputMax, outputMax );   // proportional component

   if( abs(p_Out) < outputMax ) // only add I+D when P is not saturated
   {
      float scaleFactor = (outputMax - abs(p_Out)) * invOutputMax; // soft I+D effect at extreme error

      i_Out += iGain * error * scaleFactor;     // integral component 
      i_Out = constrain( i_Out, -outputMax, outputMax );     

      d_Out = dGain * errorDelta * scaleFactor; // derivative component
      d_Out = constrain( d_Out, -outputMax, outputMax );
   }
   else
   {
      i_Out = d_Out = 0.0f;
   }

   lastError += errorDelta; 
   
   output = int( p_Out + i_Out + d_Out + fwd_Out );
   return output;
}


int  heaterPID::out()
{
   return output;
}


void  heaterPID::display()
{
   display( String( setTemp )   + " " + 
            String( probeTemp ) + " " + 
            String( p_Out )     + " " + 
            String( i_Out )     + " " + 
            String( d_Out )     + " " + 
            String( fwd_Out )   + " " + 
            String( out() )     + '\n';
}
