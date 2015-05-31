/*
 md25_diagnostic_node.cpp

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
 md25_diagnostic_node is a ROS node implementation used to receive status of MD25 board
 and publish diagnostics message.

 Subscribes:
 - status (md25_msgs/StampedStatus): the MD25 status.
 
 Publish:
 - /diagnostics (diagnostic_msgs/DiagnosticArray): the diagnostics topic.
 */
 
#include <ros/ros.h>
#include <md25_msgs/StampedStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// The pointer to diagnostics message publisher
ros::Publisher * diagnosticsMessagePublisherPtr;

void statusCallback(const md25_msgs::StampedStatus::ConstPtr & statusMessage) {

	// Calculate real motor voltage and current
	double motorVoltage = 0.1 * statusMessage->status.voltage;
	double motor1Current = 0.1 * statusMessage->status.current1;
	double motor2Current = 0.1 * statusMessage->status.current2;

	// Format real motor voltage and current
	char motorVoltageChars[10];
	sprintf(motorVoltageChars, "%g V", motorVoltage);
	char motor1CurrentChars[10];
	sprintf(motor1CurrentChars, "%g A", motor1Current);
	char motor2CurrentChars[10];
	sprintf(motor2CurrentChars, "%g A", motor2Current);

	// Create diagnostics message
	diagnostic_msgs::DiagnosticArray diagnosticsMessage;
	diagnosticsMessage.header.stamp = statusMessage->header.stamp;	
	diagnosticsMessage.status.resize(1);
	diagnosticsMessage.status[0].level = (statusMessage->status.reachable) ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
	diagnosticsMessage.status[0].name = "MD25";
	diagnosticsMessage.status[0].message = (statusMessage->status.reachable) ? "OK" : "Unreachable";
	diagnosticsMessage.status[0].hardware_id = "md25";
	diagnosticsMessage.status[0].values.resize(3);
	diagnosticsMessage.status[0].values[0].key = "Motor voltage";
	diagnosticsMessage.status[0].values[0].value = motorVoltageChars;
	diagnosticsMessage.status[0].values[1].key = "Motor 1 current";
	diagnosticsMessage.status[0].values[1].value = motor1CurrentChars;
	diagnosticsMessage.status[0].values[2].key = "Motor 2 current";
	diagnosticsMessage.status[0].values[2].value = motor2CurrentChars;

	// Publish diagnostics message
	diagnosticsMessagePublisherPtr->publish(diagnosticsMessage);

}

int main(int argc, char** argv) {

	// Init ros
	ros::init(argc, argv, "md25_diagnostic_node");

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
