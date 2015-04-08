/*
 types.cpp

 Copyright (C) 2015 Alessandro Francescon
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef _TYPES_H_
#define _TYPES_H_

#include <math.h>

// The 3-elements double vector
struct double_vector3 {

   double x;
   double y;
   double z;

   double_vector3(double x = 0, double y = 0, double z = 0): x(x), y(y), z(z) {
   }

   double_vector3 operator+(const double_vector3& v) const {
      return double_vector3(x + v.x, y + v.y, z + v.z);
   }

   double_vector3 operator-(const double_vector3& v) const {
      return double_vector3(x - v.x, y - v.y, z - v.z);
   }

   double_vector3 operator*(const double b) const {
      return double_vector3(x * b, y * b, z * b);
   }

   double_vector3 operator/(const double b) const {
      return double_vector3(x / b, y / b, z / b);
   }

   double_vector3 powV(const double b) const {
      return double_vector3(pow(x, b), pow(y, b), pow(z, b));
   }

};

// The 3-elements int16_t vector
struct int16_vector3 {

   int16_t x;
   int16_t y;
   int16_t z;

   int16_vector3(int16_t x = 0, int16_t y = 0, int16_t z = 0) : x(x), y(y), z(z) {
   }

   double_vector3 operator*(const double b) const {
      return double_vector3(x * b, y * b, z * b);
   }

   double_vector3 operator/(const double b) const {
      return double_vector3(x / b, y / b, z / b);
   }

};

#endif
