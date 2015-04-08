/*
 MPU9150.h

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


#ifndef _MPU9150_H_
#define _MPU9150_H_

#include <stdint.h>
#include <math.h>
#include <types.h>
#include <vector>

// The gravity acceleration in m/s~2
#define G 9.80665

// The accel, gyro and mag sensitivity scale factor
#define ACCEL_SSF(fs) (16348.0 / (fs + 1) / G)
#define GYRO_SSF(fs) (131.0 / (fs + 1) * 180 / M_PI)

class MPU9150 {

   public:

      // Create an MPU9150
      MPU9150();

      /*
       * Convert acceleration from LSB to m/s^2 based on given MPU9150 full scale range setup
       * accelLSBPtr the pointer to acceleration in LSB
       * accelM2SPtr the pointer to acceleration in m/s^2
       * accelFS the accel full scale range
       */
      void accelLSB2MS2(int16_vector3 * accelLSBPtr, double_vector3 * accelMS2Ptr, uint8_t accelFS);

      /*
       * Convert gyro angular velocity from LSB to rad/s based on given MPU9150 full scale range setup
       * gyroLSBPtr the pointer to angular velocity in LSB
       * gyroRSPtr the pointer to angular speed in rad/s
       * the gyro full scale range
       */
      void gyroLSB2RS(int16_vector3 * gyroLSBPtr, double_vector3 * gyroRSPtr, uint8_t gyroFS);

};

#endif
