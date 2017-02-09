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
   
      heaterPID( float Hz );
      
      void setFwdGain( float fGain );
      void setProGain( float pGain );
      void setIntGain( float iGain );
      void setDrvGain( float dGain );
      void setAllGain( float fGain, float pGain, float iGain, float dGain );
      
      void in( float setTemp, float probeTemp, float ambTemp);
      int  out();
   
   
   private:
   
      float fwdGain, proGain, intGain, drvGain;
      
      float intBucket;
      
      float fwdComponent;
      
      float sampleRateHz;
      
      float lastTemp, setPointTemp;
      
      int output;

}


void heaterPID::heaterPID( float Hz )
{
   sampleRateHz = Hz;
}


void heaterPID::setFwdGain( float fGain )
{
   fwdGain = fGain;
}


void heaterPID::setProGain( float pGain )
{
   proGain = pGain;
}


void heaterPID::setIntGain( float iGain )
{
   intGain = iGain / sampleRateHz;
}


void heaterPID::setDrvGain( float dGain )
{
   drvGain = dGain * sampleRateHz;
}


void heaterPID::setAllGain( float fGain, float pGain, float iGain, float dGain )
{
   setFwdGain( fGain );
   setProGain( pGain );
   setIntGain( iGain );
   setDrvGain( dGain );
}


void heaterPID::setPoint( float setTemp, float ambTemp)
{
   setPointTemp = setTemp;
   
   // Feed forward component scales with the square of the temperature delta
   float delta = setTemp - ambTemp;
   fwdComponent = fwdGain * delta * delta;
}


void heaterPID::in( float probeTemp )
{
   float result, error;
   
   result = fwdComponent;
   
   // PID components
   error = setTemp - probeTemp;
   
   result += proGain * error;                      // proportional component
   result += drvGain * ( lastTemp - probeTemp );   // derivative component
   result  = constrain( result, 0.0f, 100.0f );
   
   intBucket += intGain * error;
   intBucket  = constrain( intBucket, -result, 100.0f - result );  // prevent bucket from accumulating past saturation point
   result    += intBucket;

   lastTemp = probeTemp;
   
}


int  heaterPID::out()
{
   
}

