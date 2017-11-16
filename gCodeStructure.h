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


struct gCode_state_machine_t
{
   byte G[17] = {0, 0, 17, 90, 0, 94, 21, 40, 49, 80, 98, 50, 54, 0, 0, 64, 69}; // G code groups are 0 - 16
   int M;
   
   float A, B, C, D, E, F, H, I, J, K, L, N, P, Q, R, S, T, U, V, W, X, Y, Z; // G, M, O intentionally omitted 

   float startX, startY, startZ;

   bool newAxisMove, newExtruderMove, newMcode;
 
} gCode;


void gCodeSetPosition(float x, float y, float z, float e)
{
   gCode.X = x;
   gCode.Y = y;
   gCode.Z = z;
   gCode.E = e;
}


void gCodeSetX(float x)
{
   gCode.X = x;
}


void gCodeSetY(float y)
{
   gCode.Y = y;
}


void gCodeSetZ(float z)
{
   gCode.Z = z;
}


void gCodeSetE(float e)
{
   gCode.E = e;
}
