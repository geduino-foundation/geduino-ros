/*
 rplidar_node.cpp

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
 rplidar_node publish scan data from RoboPeak RPLidar.

 Publish:
 - scan (sensor_msgs/LaserScan): the scanned data;
 - /diagnostics (diagnostic_msgs/DiagnosticArray): the diagnostic information.

 Parameters:
 - serial_port, the serial port RPLidar is connected to (default: /dev/ttyUSB0);
 - frame_id, the frame laser data are referred (default: base_laser);
 - diagnostic_frequency, the diagnostic update frequency in Hz. It must be less or equals than
   odometry_frequency (default: 1);
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_srvs/Empty.h>
#include "rplidar.h"

#define SERIAL_BAUD_RATE 115200

#define RANGE_MIN 0.15
#define RANGE_MAX 6.00

#define STATUS_OK diagnostic_msgs::DiagnosticStatus::OK
#define STATUS_WARN diagnostic_msgs::DiagnosticStatus::WARN
#define STATUS_ERROR diagnostic_msgs::DiagnosticStatus::ERROR
#define STATUS_STALE diagnostic_msgs::DiagnosticStatus::STALE

using namespace rp::standalone::rplidar;

// The pouinter to rplidar driver
RPlidarDriver * rpLidarDriver = NULL;

// The rp lidar device info pointer
_rplidar_response_device_info_t * rpLidarDeviceInfoPtr = NULL;

// The laser scan publisher pointer
ros::Publisher * laserScanMessagePublisherPtr;

// The diagnostics message publisher pointer
ros::Publisher * diagnosticsMessagePublisherPtr;

// Node parameter
std::string frameId;
std::string serialPort;
double diagnosticsFrequency;

void publishLaserScan(rplidar_response_measurement_node_t * nodes, 
                  size_t nodeCount, ros::Time scanStartTime,
                  ros::Time scanEndTime) {

	// Create laser scan message
	sensor_msgs::LaserScan laserScanMessage;

	// Set laser scan message header
	laserScanMessage.header.stamp = scanStartTime;
	laserScanMessage.header.frame_id = frameId;

	// Get scan duration
	ros::Duration scanTime = scanEndTime - scanStartTime;

	// Set scan time
	laserScanMessage.scan_time = scanTime.toSec();
	laserScanMessage.time_increment = scanTime.toSec() / nodeCount;

	// Set scan angle
	laserScanMessage.angle_min = -M_PI;
	laserScanMessage.angle_max = M_PI;
	laserScanMessage.angle_increment = (2 * M_PI) / 360;

	// Set scan range limit
	laserScanMessage.range_min = RANGE_MIN;
	laserScanMessage.range_max = RANGE_MAX;

	// Resize ranges and intensities
	laserScanMessage.ranges.resize(360);
	laserScanMessage.intensities.resize(360);

	for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++) {

		// Get angle
		float angle = (float) (nodes[nodeIndex].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

		// Get range index
		int rangeIndex = (int) angle;

		// Normalize index in [0, 360[ range
		while (rangeIndex >= 360) {
			rangeIndex -= 360;
		}

		if (nodes[nodeIndex].distance_q2 != 0) {

			// Get distance and intensity
			float distance = (float) nodes[nodeIndex].distance_q2 / 4.0f / 1000;
			float syncQuality = (float) (nodes[nodeIndex].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

			// Set ranges and intensities in laser scan message
			laserScanMessage.ranges[rangeIndex] = distance;
			laserScanMessage.intensities[rangeIndex] = syncQuality;

		} else {

			// Set ranges and intensities in laser scan message
			laserScanMessage.ranges[rangeIndex] = std::numeric_limits<float>::infinity();
			laserScanMessage.intensities[rangeIndex] = 0;

		}

	}

	// Publish laser scan message
	laserScanMessagePublisherPtr->publish(laserScanMessage);

}

void publishDiagnostics(uint8_t level, std::string message) {

	// Create diagnostics message
	diagnostic_msgs::DiagnosticArray diagnosticsMessage;
	diagnosticsMessage.header.stamp = ros::Time::now();
	diagnosticsMessage.status.resize(1);
	diagnosticsMessage.status[0].level = level;
	diagnosticsMessage.status[0].name = "RPLidar";
	diagnosticsMessage.status[0].message = message.c_str();
	diagnosticsMessage.status[0].hardware_id = "rplidar";

	if (rpLidarDeviceInfoPtr) {

		// Get values as char array
		char modelChars[1];
		sprintf(modelChars, "%x", rpLidarDeviceInfoPtr->model);
		char firmwareVersionChars[3];
		sprintf(firmwareVersionChars, "%u.%u", (uint8_t) rpLidarDeviceInfoPtr->firmware_version >> 8, (uint8_t) rpLidarDeviceInfoPtr->firmware_version);
		char hardwareVersionChars[1];
		sprintf(hardwareVersionChars, "%x", rpLidarDeviceInfoPtr->hardware_version);
		char serialNumberChars[32];
		for (uint8_t index = 0; index < 16; index++) {
			sprintf(&serialNumberChars[index * 2], "%x", rpLidarDeviceInfoPtr->serialnum[15 - index]);
		}

		diagnosticsMessage.status[0].values.resize(4);
		diagnosticsMessage.status[0].values[0].key = "Model";
		diagnosticsMessage.status[0].values[0].value = modelChars;
		diagnosticsMessage.status[0].values[1].key = "Firmware version";
		diagnosticsMessage.status[0].values[1].value = firmwareVersionChars;
		diagnosticsMessage.status[0].values[2].key = "Hardware version";
		diagnosticsMessage.status[0].values[2].value = hardwareVersionChars;
		diagnosticsMessage.status[0].values[3].key = "Serial number";
		diagnosticsMessage.status[0].values[3].value = serialNumberChars;

	} else {

		// No values
		diagnosticsMessage.status[0].values.resize(0);

	}

	// Publish diagnostics message
	diagnosticsMessagePublisherPtr->publish(diagnosticsMessage);

}

bool checkHealthStatus() {

	rplidar_response_device_health_t health;

	// Get rp lidar health
	u_result opResult = rpLidarDriver->getHealth(health);

	// Check rp lidar health
	if (IS_OK(opResult)) {

		// Check health status
		if (health.status == RPLIDAR_STATUS_OK) {

			// Log
			ROS_INFO("rp lidar health status ok");

			return true;

		} else if (health.status == RPLIDAR_STATUS_WARNING) {

			// Log
			ROS_WARN("rp lidar health status is warning: continue anyway but it must be checked...");

			return true;

		} else if (health.status == RPLIDAR_STATUS_ERROR) {

			// Log
			ROS_ERROR("rp lidar health status error");

			return false;

		} else {

			// Log
			ROS_ERROR("rp lidar health status unknown: %d", health.status);

			return false;

		}

	} else {

		// Log
		ROS_ERROR("cannot retrieve rp lidar health status. Operation result: %x", opResult);

		return false;

	}

}

void startScan() {

	if (rpLidarDriver) {

        // Start motor
        rpLidarDriver->startMotor();

		// Start scan
		rpLidarDriver->startScan();

		// Log
		ROS_INFO("waiting for rp lidar motor to stabilize...");

		// Sleep for 1 sec to leave time motor speed to stabilize
		usleep(1000 * 1000);

	} else {

		// Log
		ROS_ERROR("cannot start scan: rp lidar driver was null");

	}

}

void stopScan() {

	if (rpLidarDriver) {

		// Stop scan
		rpLidarDriver->stop();

        // Stop motor
        rpLidarDriver->stopMotor();

	} else {

		// Log
		ROS_ERROR("cannot stop scan: rp lidar driver was null");

	}

}

void dispose() {

	if (rpLidarDriver) {

		// Log
		ROS_INFO("disposing rp lidar driver...");

		// Dispose rp lidar driver
		RPlidarDriver::DisposeDriver(rpLidarDriver);

	}

}

int main(int argc, char * argv[]) {

	// Init node handle
	ros::init(argc, argv, "rplidar_node");

	// Get private node handle
	ros::NodeHandle privateNodeHandle("~");

	// Get node params
	privateNodeHandle.param<std::string>("frame_id", frameId, "base_laser");
    privateNodeHandle.param<std::string>("serial_port", serialPort, "/dev/ttyUSB0");
    privateNodeHandle.param("diagnostics_frequency", diagnosticsFrequency, 1.0);

	// Log
	ROS_INFO("framed id: %s", frameId.c_str());
	ROS_INFO("serial port: %s", serialPort.c_str());
    ROS_INFO("diagnostics update frequency: %g Hz", diagnosticsFrequency);

	// Get node handle
	ros::NodeHandle nodeHandle;

	// Create disgnostics message publisher
    ros::Publisher diagnosticsMessagePublisher = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 20);
	diagnosticsMessagePublisherPtr = & diagnosticsMessagePublisher;

	// Log
	ROS_INFO("setting up rp lidar driver...");

	// Create rp lidar driver
	rpLidarDriver = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	// Check if rp lidar driver was created
	if (!rpLidarDriver) {

		// Log
		ROS_ERROR("cannot create driver. Exiting...");

		// Publish diagnostics
		publishDiagnostics(STATUS_ERROR, "Cannot create driver");

		return -2;

	}

	// Connect rp lidar driver
	if (IS_FAIL(rpLidarDriver->connect(serialPort.c_str(), (_u32) SERIAL_BAUD_RATE))) {

		// Log
		ROS_ERROR("cannot bind serial port %s. Exiting...", serialPort.c_str());

		// Publish diagnostics
		publishDiagnostics(STATUS_ERROR, "Cannot bind serial port");

		// Dispose resources
		dispose();

		return -1;

	}

	// Check rp lidar deriver health status
	if (!checkHealthStatus()) {

		// Log
		ROS_ERROR("rp lidar health status check failed. Exiting...");

			// Publish diagnostics
		publishDiagnostics(STATUS_ERROR, "Bad health status");

		// Dispose resources
		dispose();

		return -1;

	}


	_rplidar_response_device_info_t rpLidarDeviceInfo;

	// Get rp lidar device info
	if (IS_OK(rpLidarDriver->getDeviceInfo(rpLidarDeviceInfo))) {

		// Set pointer to rp lidar device info
		rpLidarDeviceInfoPtr = & rpLidarDeviceInfo;

	} else {

		// Log
		ROS_ERROR("cannot get rp lidar device info");

	}

	// Log
	ROS_INFO("all ready: starting scan...");

	// Publish diagnostics
	publishDiagnostics(STATUS_OK, "OK");

	// Start scan
	startScan();

	// Create laser scan message publisher
 	ros::Publisher laserScanMessagePublisher = nodeHandle.advertise<sensor_msgs::LaserScan>("scan", 1000);
	laserScanMessagePublisherPtr = & laserScanMessagePublisher;

    // Define diagnostic duration
    ros::Duration diagnosticsDuration(1 / diagnosticsFrequency);
    ros::Time lastDiagnosticsUpdateTime = ros::Time::now();

	u_result opResult = RESULT_OK;

	while (ros::ok()) {

		rplidar_response_measurement_node_t nodes[720];
		size_t nodeCount = 720;

		// Get scan start time
		ros::Time scanStartTime = ros::Time::now();

		// Grab scan data
		opResult = rpLidarDriver->grabScanData(nodes, nodeCount);

		// Get scan end time
		ros::Time scanEndTime = ros::Time::now();

		if (opResult == RESULT_OK) {

			// Ascend scan data
			opResult = rpLidarDriver->ascendScanData(nodes, nodeCount);

			if (opResult == RESULT_OK) {

				// Publish laser scan
				publishLaserScan(nodes, nodeCount, scanStartTime, scanEndTime);

			} else {

				// Log
				ROS_WARN("cannot ascend scan data. Operation result: %x", opResult);

				// Publish diagnostics
				publishDiagnostics(STATUS_WARN, "Cannot ascend scan data");

			}


		} else {

			// Log
			ROS_WARN("cannot grab scan data. Operation result: %x", opResult);

			// Publish diagnostics
			publishDiagnostics(STATUS_WARN, "Cannot grab scan data");

		}

        // Get now
        ros::Time now = ros::Time::now();

        if (opResult == RESULT_OK && (now - lastDiagnosticsUpdateTime) > diagnosticsDuration) {

            // Publish diagnostics
            publishDiagnostics(STATUS_OK, "OK");

            // Reset last diagnostics update time
            lastDiagnosticsUpdateTime = now;

        }

        // Ros spin
        ros::spinOnce();

	}

	// Stop scan
	stopScan();

	// Dispose resources
	dispose();

	return 0;

}
