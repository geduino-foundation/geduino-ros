/*
 geduino_diagnostic_node.cpp

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
 geduino_diagnostic_node is a ROS node implementation used to receive status of Geduino
 and publish diagnostics message.

 Subscribes:
 - status (geduino_diagnostic_msgs/StampedStatus): the Geduino status.
 
 Publish:
 - /diagnostics (diagnostic_msgs/DiagnosticArray): the diagnostics topic.

 Parameters:
 - samx8_load_warning_threshold, the warning value for SAMx8 average load,
   in percent (default: 10.0);
 - samx8_load_critical_threshold, the critical value for SAMx8 average load,
   in percent (default: 50.0);
 - samx8_delay_warning_threshold, the warning value for SAMx8 average delay,
   in percent (default: 50.0);
 - samx8_delay_critical_threshold, the critical value for SAMx8 average delay,
   in percent (default: 100.0);
 */
 
#include <ros/ros.h>
#include <geduino_diagnostic_msgs/StampedStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#define LIPOS4_WARNING_THRESHOLD 13.6
#define LIPOS4_CRITICAL_THRESHOLD 12.4

// The pointer to diagnostics message publisher
ros::Publisher * diagnosticsMessagePublisherPtr;

// Node parameters
double samx8LoadWarningThreshold;
double samx8LoadCriticalThreshold;
double samx8DelayWarningThreshold;
double samx8DelayCriticalThreshold;

void statusCallback(const geduino_diagnostic_msgs::StampedStatus::ConstPtr & statusMessage) {

	// Create diagnostics message
	diagnostic_msgs::DiagnosticArray diagnosticsMessage;
	diagnosticsMessage.header.stamp = statusMessage->header.stamp;	
	diagnosticsMessage.status.resize(2);

	// Power diagnostic

	// Calculate power voltage
	double powerVoltage = 0.001 * geduino_diagnostic_msgs::Power::RAW_TO_MILLIVOLT * statusMessage->status.power.raw_voltage;
       
	// Format power voltage
	char powerVoltageChars[15];
	sprintf(powerVoltageChars, "%g V", powerVoltage);
	
	if (powerVoltage < LIPOS4_CRITICAL_THRESHOLD) {

		// Status ERROR, message CRITICAL
		diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
		diagnosticsMessage.status[0].message = "Voltage below critical threshold";

	} else if (powerVoltage < LIPOS4_WARNING_THRESHOLD) {

		// Status WARN, message WARNING
		diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::WARN;
		diagnosticsMessage.status[0].message = "Voltage below warning threshold";

	} else {

		// Status OK, message OK
		diagnosticsMessage.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
		diagnosticsMessage.status[0].message = "OK";

	}

	diagnosticsMessage.status[0].name = "Power";
	diagnosticsMessage.status[0].hardware_id = "power";
	diagnosticsMessage.status[0].values.resize(2);
	diagnosticsMessage.status[0].values[0].key = "Voltage";
	diagnosticsMessage.status[0].values[0].value = powerVoltageChars;
	diagnosticsMessage.status[0].values[1].key = "Type";
	diagnosticsMessage.status[0].values[1].value = (statusMessage->status.power.type == geduino_diagnostic_msgs::Power::TYPE_LIPO_4S) ? "LiPo 4S" : "AC adapter";

	// SAMx8 diagnostic

	// Calculate average load and delays
        double averageLoad = 100 * statusMessage->status.proc_stat.used_cycles / statusMessage->status.proc_stat.idle_cycles;
	double encodersMotion9AverageDelay = 0.1 * statusMessage->status.proc_stat.encoders_motion9_delay.sum / statusMessage->status.proc_stat.encoders_motion9_delay.counter;
	double diagnosticAverageDelay = 0.1 * statusMessage->status.proc_stat.diagnostic_delay.sum / statusMessage->status.proc_stat.diagnostic_delay.counter;

	// Format average load and delays
	char averageLoadChars[15];
	sprintf(averageLoadChars, "%.2f %%", averageLoad);
	char encodersMotion9AverageDelayChars[15];
	sprintf(encodersMotion9AverageDelayChars, "%.2f millis", encodersMotion9AverageDelay);
	char diagnosticAverageDelayChars[15];
	sprintf(diagnosticAverageDelayChars, "%.2f millis", diagnosticAverageDelay);

	if (averageLoad > samx8LoadCriticalThreshold ||
		encodersMotion9AverageDelay > samx8DelayCriticalThreshold ||
		diagnosticAverageDelay > samx8DelayCriticalThreshold) {

		// Status ERROR, message LOAD or DELAY CRITICAL
		diagnosticsMessage.status[1].level = diagnostic_msgs::DiagnosticStatus::ERROR;
		diagnosticsMessage.status[1].message = "Load and/or delay over critical threshold";

	} else if (averageLoad > samx8LoadWarningThreshold ||
		encodersMotion9AverageDelay > samx8DelayWarningThreshold ||
		diagnosticAverageDelay > samx8DelayWarningThreshold) {

		// Status WARN, message LOAD or DELAY WARNING
		diagnosticsMessage.status[1].level = diagnostic_msgs::DiagnosticStatus::WARN;
		diagnosticsMessage.status[1].message = "Load and/or delay over warning threshold";

	} else {

		// Status OK, message OK
		diagnosticsMessage.status[1].level = diagnostic_msgs::DiagnosticStatus::OK;
		diagnosticsMessage.status[1].message = "OK";

	}

	diagnosticsMessage.status[1].name = "SAMx8";
	diagnosticsMessage.status[1].hardware_id = "samx8";
	diagnosticsMessage.status[1].values.resize(3);
	diagnosticsMessage.status[1].values[0].key = "Average load";
	diagnosticsMessage.status[1].values[0].value = averageLoadChars;
	diagnosticsMessage.status[1].values[1].key = "Encoders and Motion9 average delay";
	diagnosticsMessage.status[1].values[1].value = encodersMotion9AverageDelayChars;
	diagnosticsMessage.status[1].values[2].key = "Diagnostic average delay";
	diagnosticsMessage.status[1].values[2].value = diagnosticAverageDelayChars;

	// Publish diagnostics message
	diagnosticsMessagePublisherPtr->publish(diagnosticsMessage);

}

int main(int argc, char** argv) {

	// Init ros
	ros::init(argc, argv, "geduino_diagnostic_node");

	// Get private node handle
	ros::NodeHandle privateNodeHandle("~");

	// Get node params
	privateNodeHandle.param("samx8_load_warning_threshold", samx8LoadWarningThreshold, 10.0);
	privateNodeHandle.param("samx8_load_critical_threshold", samx8LoadCriticalThreshold, 50.0);
	privateNodeHandle.param("samx8_delay_warning_threshold", samx8DelayWarningThreshold, 50.0);
	privateNodeHandle.param("samx8_delay_critical_threshold", samx8DelayCriticalThreshold, 100.0);

	// Log
	ROS_INFO("SamX8 load warning threshold %g", samx8LoadWarningThreshold);
	ROS_INFO("SamX8 load critical threshold %g", samx8LoadCriticalThreshold);
	ROS_INFO("SamX8 delay warning threshold %g", samx8DelayWarningThreshold);
	ROS_INFO("SamX8 delay critical threshold %g", samx8DelayCriticalThreshold);

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
