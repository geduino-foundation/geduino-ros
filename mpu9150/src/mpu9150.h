/*
 mpu9150.h

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

#ifndef _INVMPU9150_H_
#define _INVMPU9150_H_

#define MPU9150_OK							0
#define MPU9150_ERROR							-1
#define MPU9150_INIT_MPU_INIT_ERROR					-2
#define MPU9150_INIT_MPU_SET_SENSORS_ERROR				-3
#define MPU9150_INIT_MPU_SET_GYRO_FSR_ERROR				-4
#define MPU9150_INIT_MPU_SET_ACCEL_FSR_ERROR				-5
#define MPU9150_INIT_MPU_CONFIGURE_FIFO_ERROR				-6
#define MPU9150_INIT_MPU_SET_SAMPLE_RATE_ERROR				-7
#define MPU9150_INIT_DMP_LOAD_MOTION_DRIVER_FIRMWARE_ERROR		-8
#define MPU9150_INIT_DMP_SET_ORIENTATION_ERROR				-9
#define MPU9150_INIT_DMP_ENABLE_FEATURES_ERROR				-10
#define MPU9150_INIT_DMP_SET_FIFO_RATE_ERROR				-11
#define MPU9150_INIT_DMP_SET_STATE_ERROR				-12

// The DMP fifo data struct
typedef struct {
	float gyro[3];
	float accel[3];
	float quat[4];
} dmpFifoData_t;

class INVMPU9150 {

	public:

		// Create a new INVMPU9150
		INVMPU9150();

		// Init the INVMPU9150 at given I2C bus and upload DMP driver to work with given frequency.
		// It return MPU9150_OF if init process succes, a different code otherwise.
		int init(int i2cBus, int frequency);

		// Dispose the INVMPU9150.
		// It return MPU9150_OF if dispose process succes, a different code otherwise.
		int dispose();

		// Return 1 if DMP data in available on FIFO. Other value otherwise.
		int dmpFifoDataReady();

		// Read DMP data from FIFO and return MPU9150_OK if the operation success. Return other value otherwise.
		int dmpReadFifoData(dmpFifoData_t * dmpFifoData);

};

#endif
