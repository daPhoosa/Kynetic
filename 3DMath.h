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


struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};


Vec3 VectorSet( float x, float y, float z )
{
   Vec3 = a;
   a.x = x;
   a.y = y;
   a.z = z;
   return a;
}


Vec3 VectorSub(Vec3 a, const Vec3& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}


Vec3 VecCrossProd(const Vec3& L, const Vec3& R) // cross product of 3D vectors 
{ 
    Vec3 a;
    a.x = L.y * R.z - L.z * R.y;
    a.y = L.z * R.x - L.x * R.z;
    a.z = L.x * R.y - L.y * R.x;
    return a;
}


float VecMagSq(const Vec3& vec)
{
   return vec.x * vec.x + vec.y * vec.y + vec.z + vec.z;
}


float VecDotProd(const Quat& L, const Quat& R)
{
    return L.w * R.w + L.x * R.x + L.y * R.y + L.z * R.z;
}
