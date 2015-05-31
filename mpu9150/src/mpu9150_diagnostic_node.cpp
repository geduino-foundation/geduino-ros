/*
 mpu9150_diagnostic_node.cpp

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
 mpu9150_diagnostic_node is a ROS node implementation used to receive status of MPU9150
 and publish diagnostics message.

 Subscribes:
 - status (mpu9150_msgs/StampedStatus): the MPU9150 status.
 
 Publish:
 - /diagnostics (diagnostic_msgs/DiagnosticArray): the diagnostics topic.
 */
 
#include <ros/ros.h>
#include <mpu9150_msgs/StampedStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// The pointer to diagnostics message publisher
ros::Publisher * diagnosticsMessagePublisherPtr;

void statusCallback(const mpu9150_msgs::StampedStatus::ConstPtr & statusMessage) {

	// Format fifo overflows sum and counter and temperature
	char fifoOverflowsSumChars[10];
	sprintf(fifoOverflowsSumChars, "%u", statusMessage->status.fifo_overflows.sum);
	char fifoOverflowsCounterChars[10];
	sprintf(fifoOverflowsCounterChars, "%u", statusMessage->status.fifo_overflows.counter);
	char temperatureChars[10];
	sprintf(temperatureChars, "%.2f C", statusMessage->status.temperature);

	// Create diagnostics message
	diagnostic_msgs::DiagnosticArray diagnosticsMessage;
	diagnosticsMessage.header.stamp = statusMessage->header.stamp;	
	diagnosticsMessage.status.resize(1);

	if (statusMessage->status.initialized) {

		if (statusMessage->status.fifo_overflows.sum == 0) {
			
			// Level OK, message OK
			diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
			diagnosticsMessage.status[0].message = "OK";

		} else {

			// Level WARN, message OVERFLOW
			diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::WARN;
			diagnosticsMessage.status[0].message = "FIFO overflows";

		}

	} else {

		// Level ERROR, message NOT INITIALIZED
		diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
		diagnosticsMessage.status[0].message = "Initialization failed";

	}

	diagnosticsMessage.status[0].name = "MPU9150";
	diagnosticsMessage.status[0].hardware_id = "mpu9150";
	diagnosticsMessage.status[0].values.resize(3);
	diagnosticsMessage.status[0].values[0].key = "FIFO overflows";
	diagnosticsMessage.status[0].values[0].value = fifoOverflowsSumChars;
	diagnosticsMessage.status[0].values[1].key = "FIFO access counter";
	diagnosticsMessage.status[0].values[1].value = fifoOverflowsCounterChars;
	diagnosticsMessage.status[0].values[2].key = "Temperature";
	diagnosticsMessage.status[0].values[2].value = temperatureChars;

	// Publish diagnostics message
	diagnosticsMessagePublisherPtr->publish(diagnosticsMessage);

}

int main(int argc, char** argv) {

	// Init ros
	ros::init(argc, argv, "mpu9150_diagnostic_node");

	// Get node handle
	ros::NodeHandle nodeHandle;

	// Create status message subscriber
	ros::Subscriber statusMessageSubscriber = nodeHandle.subscribe("status", 20, statusCallback);

	// Create diagnostics message publisher
	ros::Publisher diagnosticsMessagePublisher = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 20);
	diagnosticsMessagePublisherPtr = & diagnosticsMessagePublisher;

	// Spin
	ros::spin();

	return 0;

}
