/*
 mpu9150.cpp

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


#include "mpu9150.h"

extern "C" {
        #include <linux_glue.h>
        #include <inv_mpu.h>
        #include <inv_mpu_dmp_motion_driver.h>
	#include <utility.h>
}

// The DMP fifo data in LSB unit structure
typedef struct {
        short gyro[3];
        short accel[3];
        long quat[4];
} dmpFifoDataLsb_t;

/* Public methods */

INVMPU9150::INVMPU9150() {
}

int INVMPU9150::dispose() {

	// Set dmp status to off
	if (mpu_set_dmp_state(0)) {
		return MPU9150_ERROR;
	} else {
		return MPU9150_OK;
	}

}

int INVMPU9150::init(int i2cBus, int frequency) {

	// Set linux i2c bus
	linux_set_i2c_bus(i2cBus);

	// The gyro orientation
	signed char gyro_orientation[9] = { 1, 0, 0,
					0, 1, 0,
					0, 0, 1 };

	// Init MPU
	if (mpu_init(NULL)) {
		return MPU9150_INIT_MPU_INIT_ERROR;
	}

	// Set sensors
	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
		return MPU9150_INIT_MPU_SET_SENSORS_ERROR;
	}

	// Get gyro sensitivity scale factor and convert from [LSB/(degree/s)] to [LSB/(rad/s)]
	float gyroSsfDegreePerSecond;
	if (mpu_get_gyro_sens(& gyroSsfDegreePerSecond)) {
		return MPU9150_INIT_MPU_GET_GYRO_FSR_ERROR;
	}
	gyroSsf = gyroSsfDegreePerSecond * 180 / M_PI;

	// Get accel sensitivity scale factor
	if (mpu_get_accel_sens(& accelSsf)) {
		return MPU9150_INIT_MPU_GET_ACCEL_FSR_ERROR;
	}

	// Configure FIFO
	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
		return MPU9150_INIT_MPU_CONFIGURE_FIFO_ERROR;
	}

	// Set sample rate for gyro/acc
	if (mpu_set_sample_rate(frequency)) {
		return MPU9150_INIT_MPU_SET_SAMPLE_RATE_ERROR;
	}

	// Load motion driver firmare (DMP)
	if (dmp_load_motion_driver_firmware()) {
		return MPU9150_INIT_DMP_LOAD_MOTION_DRIVER_FIRMWARE_ERROR;
	}

	// Set gyro orientation
	if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
		return MPU9150_INIT_DMP_SET_ORIENTATION_ERROR;
	}

	// Enable DMP features
  	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL
						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {
		return MPU9150_INIT_DMP_ENABLE_FEATURES_ERROR;
	}

	// Set sample rate for DMP
	if (dmp_set_fifo_rate(frequency)) {
		return MPU9150_INIT_DMP_SET_FIFO_RATE_ERROR;
	}

	// Enable DMP
	if (mpu_set_dmp_state(1)) {
		return MPU9150_INIT_DMP_SET_STATE_ERROR;
	}

	return MPU9150_OK;

}

int INVMPU9150::dmpFifoDataReady() {

	short status;

	// Get interrupt status
	if (mpu_get_int_status(&status) < 0) {
		return 0;
	}

	return (status = 0x0103);

}

int INVMPU9150::dmpReadFifoData(dmpFifoData_t * dmpFifoData) {

	unsigned long timestamp;
	short sensors;
	unsigned char more;

	// The dmp data as LSB
	dmpFifoDataLsb_t dmpFifoDataLsb;

	// Read fifo data
	if  (dmp_read_fifo(dmpFifoDataLsb.gyro, dmpFifoDataLsb.accel, dmpFifoDataLsb.quat, &timestamp, &sensors, &more) < 0) {
		return MPU9150_ERROR;
	}

	while (more) {

		// Fell behind, reading again
		if  (dmp_read_fifo(dmpFifoDataLsb.gyro, dmpFifoDataLsb.accel, dmpFifoDataLsb.quat, &timestamp, &sensors, &more) < 0) {
			return MPU9150_ERROR;
		}

	}

	// Covert gyro data to rad/s
	dmpFifoData->gyro[0] = ((float) dmpFifoDataLsb.gyro[0]) / gyroSsf;
	dmpFifoData->gyro[1] = ((float) dmpFifoDataLsb.gyro[1]) / gyroSsf;
	dmpFifoData->gyro[2] = ((float) dmpFifoDataLsb.gyro[2]) / gyroSsf;

	// Convert accel data to m/s2
	dmpFifoData->accel[0] = ((float) dmpFifoDataLsb.accel[0]) / accelSsf;
	dmpFifoData->accel[1] = ((float) dmpFifoDataLsb.accel[1]) / accelSsf;
	dmpFifoData->accel[2] = ((float) dmpFifoDataLsb.accel[2]) / accelSsf;

	// Normalize quaternion
	float norm = sqrtf((float) dmpFifoDataLsb.quat[0] * (float) dmpFifoDataLsb.quat[0] +
		(float) dmpFifoDataLsb.quat[1] * (float) dmpFifoDataLsb.quat[1] +
		(float) dmpFifoDataLsb.quat[2] * (float) dmpFifoDataLsb.quat[2] +
		(float) dmpFifoDataLsb.quat[3] * (float) dmpFifoDataLsb.quat[3]);
	dmpFifoData->quat[0] = dmpFifoDataLsb.quat[0] / norm;
	dmpFifoData->quat[1] = dmpFifoDataLsb.quat[1] / norm;
	dmpFifoData->quat[2] = dmpFifoDataLsb.quat[2] / norm;
	dmpFifoData->quat[3] = dmpFifoDataLsb.quat[3] / norm;

	return MPU9150_OK;

}







