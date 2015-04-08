/*
 MPU9150.cpp

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


#include <mpu9150.h>

MPU9150::MPU9150() {
}

void MPU9150::accelLSB2MS2(int16_vector3 * accelLSBPtr, double_vector3 * accelMS2Ptr, uint8_t accelFS) {

   // Get accel sensitivity scale factor
   double accelSSF= ACCEL_SSF(accelFS);

   // Compute acceÃl in m/^2
   accelMS2Ptr->x = accelLSBPtr->x / accelSSF;
   accelMS2Ptr->y = accelLSBPtr->y / accelSSF;
   accelMS2Ptr->z = accelLSBPtr->z / accelSSF;

}

void MPU9150::gyroLSB2RS(int16_vector3 * gyroLSBPtr, double_vector3 * gyroRSPtr, uint8_t gyroFS) {

   // Get giro sensitivity scalr factor
   double gyroSSF = GYRO_SSF(gyroFS);

   // Compute angular speed in rad/s
   gyroRSPtr->x = gyroLSBPtr->x / gyroSSF;
   gyroRSPtr->y = gyroLSBPtr->y / gyroSSF;
   gyroRSPtr->z = gyroLSBPtr->z / gyroSSF;

}
