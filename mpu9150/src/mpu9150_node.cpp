/*
 mpu9150_node.cpp
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

/**
 mpu9150_node is a ROS node implementation for MPU9150 IMU from InvenSense. 
 It publish imu message based on data fetched by the IMU.
 The MPU9150 is setup to use internal DMP algorithm.
 This implementation is based on the work of Pansenti (see
 https://github.com/Pansenti/linux-mpu950.git.
 Publish:
 - /imu (sensor_msgs/Imu): the IMU data;
 - /diagnostics (diagnostic_msgs/DiagnosticArray): the diagnostic information.
 Parameters:
 - i2c_bus, the I2C bus the MPU9150 is attached to (default: 0);
 - frequency, the DMP algoritm and IMU message publish rate in Hz. It must be between
   2 and 50  (default: 10);
- imu_frame, the frame of the MPU9150 (default: base_imu);
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "mpu9150.h"

#define STATUS_OK diagnostic_msgs::DiagnosticStatus::OK
#define STATUS_WARN diagnostic_msgs::DiagnosticStatus::WARN
#define STATUS_ERROR diagnostic_msgs::DiagnosticStatus::ERROR
#define STATUS_STALE diagnostic_msgs::DiagnosticStatus::STALE

// The ros parameters
int i2cBus;
int frequency;
std::string imuFrame;

// The imu message publisher
ros::Publisher * imuMessagePublisher;

// The diagnostics message publisher pointer
ros::Publisher * diagnosticsMessagePublisherPtr;

void publishDiagnostics(uint8_t level, std::string message) {

	// Create diagnostics message
	diagnostic_msgs::DiagnosticArray diagnosticsMessage;
	diagnosticsMessage.header.stamp = ros::Time::now();
	diagnosticsMessage.status.resize(1);
	diagnosticsMessage.status[0].level = level;
	diagnosticsMessage.status[0].name = "MPU9150";
	diagnosticsMessage.status[0].message = message.c_str();
	diagnosticsMessage.status[0].hardware_id = "MPU9150";

	// Publish diagnostics message
	diagnosticsMessagePublisherPtr->publish(diagnosticsMessage);

}

int main(int argc, char** argv) {

	// Init ros
	ros::init(argc, argv, "mpu9150_node");

	// Get node handle
	ros::NodeHandle nodeHandle;

	// Get private node handle
	ros::NodeHandle privateNodeHandle("~");

	// Get ros parameters
	privateNodeHandle.param("i2c_bus", i2cBus, 0);
	privateNodeHandle.param("frequency", frequency, 10);
	privateNodeHandle.param<std::string>("imu_frame", imuFrame, "base_imu");

	// Log
	ROS_INFO("i2c bus: %d", i2cBus);
	ROS_INFO("frequency: %d Hz", frequency);
	ROS_INFO("imu frame: %s", imuFrame.c_str());

  	// Create imu message publisher
	ros::Publisher _imuMessagePublisher = nodeHandle.advertise<sensor_msgs::Imu>("/imu", 20);;
	imuMessagePublisher = &_imuMessagePublisher;

	// Create disgnostics message publisher
	ros::Publisher diagnosticsMessagePublisher = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 20, true);
	diagnosticsMessagePublisherPtr = & diagnosticsMessagePublisher;

	// Create MPU9150
	INVMPU9150 mpu9150;

	// Log
	ROS_INFO("Initializing MPU9150...");

	// Init mpu91500
	int result = mpu9150.init(i2cBus, frequency);

	if (result) {

		// Log
		ROS_ERROR("MPU9150 init failed: %d", result);

		// Publish diagnostics
		publishDiagnostics(STATUS_ERROR, "Initialization failed");

		return -1;

	}

	// Log
	ROS_INFO("MPU9150 setup succesfully");

	// Publish diagnostics
	publishDiagnostics(STATUS_OK, "OK");

	// Define rate
	ros::Rate rate(frequency);

	while (ros::ok()) {

		dmpFifoData_t dmpFifoData;

		if (mpu9150.dmpFifoDataReady()) {

			if (mpu9150.dmpReadFifoData(&dmpFifoData) < 0) {

				// Log
				ROS_ERROR("dmp_read_fifo() failed");

				// Publish diagnostics
				publishDiagnostics(STATUS_ERROR, "Cannot read data from FIFO");

			} else {

				// Create imu message
				sensor_msgs::Imu imuMessage;
				imuMessage.header.stamp = ros::Time::now();;
				imuMessage.header.frame_id = imuFrame;

				// Compute angular velocity
				imuMessage.angular_velocity.x = dmpFifoData.gyro[0];
				imuMessage.angular_velocity.y = dmpFifoData.gyro[1];
				imuMessage.angular_velocity.z = dmpFifoData.gyro[2];

				// Set linear acceleration
				imuMessage.linear_acceleration.x = dmpFifoData.accel[0];
				imuMessage.linear_acceleration.y = dmpFifoData.accel[1];
				imuMessage.linear_acceleration.z = dmpFifoData.accel[2];

				// Get orientation
				imuMessage.orientation.x = dmpFifoData.quat[0];
				imuMessage.orientation.y = dmpFifoData.quat[1];
				imuMessage.orientation.z = dmpFifoData.quat[2];
				imuMessage.orientation.w = dmpFifoData.quat[3];

				// Publish imu message
				imuMessagePublisher->publish(imuMessage);

			}

		} else {

			// Log
			ROS_WARN("no data available");

			// Publish diagnostics
			publishDiagnostics(STATUS_WARN, "No data available");

		}

		// Spin once
		ros::spinOnce();

		// Sleep
		rate.sleep();

	}

	// Disable DMP
	if (mpu9150.dispose()) {

		// Log
		ROS_ERROR("mpu_set_dmp_state(0) failed");

	}

	return 0;

}


