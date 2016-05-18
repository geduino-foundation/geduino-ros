/*
 md25_node.cpp

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
 md25_node is a ROS node implementation for  MD25 boards. It publish odometry transformation 
 and topic to the navigation stack and subscribe cmd_vel to set whhels speed on MD25.

 The covariance model used get from work of Linsday KLEEMAN of Monash University (technical
 report MECSE-95-1-1995).
 This model assume that variance of travelled space for a wheel is proportial to travelled 
 space by a constant.

 Subscribes:
 - /cmd_vel (geometry_msgs/Twist): the velocity command.

 Publish:
 - /odom (nav_msgs/Odometry): the odometry topic;
 - /diagnostics (diagnostic_msgs/DiagnosticArray): the diagnostic information.

 Parameters:
 - port, the serial port which MD25 is connected to (default: /dev/ttymxc2);
 - baudrate, the baudrate to be used (default: 38400);
 - timeout, the serial timeout in millis (default: 250);
 - base_frame, the frame attached to robot base, i.e. broadcasted transformation child
   frame (default: base_link);
 - odom_frame, : odometry frame, i.e. broadcasted transformation frame (default: odom);
 - odometry_frequency, the odometry update frequency in Hz (default: 10);
 - diagnostic_frequency, the diagnostic ypdate frequency in Hz. It must be less or equals than
   odometry_frequency (default: 1);
 - encoder_sensitivity1: the encoder 1 sensitivity in LSB / rad. It represent the inverse
   of wheel shaft rotation angle for unitary encoder increment. (default: 0.00872639,
   valid using EMG30 motor);
 - encoder_sensitivity2: the encoder 2 sensitivity in LSB / rad. It represent the inverse
   of wheel shaft rotation angle for unitary encoder increment. (default: 0.00872639,
   valid using EMG30 motor);
 - speed_sensitivity1: the speed 1 sensitivity in LSB / (rad/s). It represent the inverse
   of wheel shaft rotation speed for unitary speed input. (default: 12.4620, valid using
   EMG30 motor);
 - speed_sensitivity2: the speed 2 sensitivity in LSB / (rad/s). It represent the inverse
   of wheel shaft rotation speed for unitary speed input. (default: 12.4620, valid using
   EMG30 motor);
 - wheel_diameter1: the wheel 1 diameter in m (default: 0.1);
 - wheel_diameter2: the wheel 2 diameter in m (default: 0.1);
 - wheel_base: the wheel base in m (default: 0.3);
 - cov_K1: the constant of covariance model for wheel 1;
 - cov_K2: the constant of covariance model for wheel 2;
 - cov_radius_threshold: the radius threshold to distinguish between straight andarc path.
 - yaw_speed_variance: the variance of yaw speed (default: 0.06)
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <math.h>
#include <md25.h>
#include <odometry.h>

#define STATUS_OK diagnostic_msgs::DiagnosticStatus::OK
#define STATUS_WARN diagnostic_msgs::DiagnosticStatus::WARN
#define STATUS_ERROR diagnostic_msgs::DiagnosticStatus::ERROR
#define STATUS_STALE diagnostic_msgs::DiagnosticStatus::STALE

// The serial parameters
std::string port;
int baudrate;
int timeout;

// The ros parameters
std::string baseFrame;
std::string odomFrame;

// Other parameters
double odometryFrequency;
double diagnosticsFrequency;
double encoderSensitivity1;
double encoderSensitivity2;
double speedSensitivity1;
double speedSensitivity2;
double wheelDiameter1;
double wheelDiameter2;
double wheelBase;
double covK1;
double covK2;
double covRTres;
double yawSpeedVariance;

// The last encoder values
uint32_t lastEncoder1 = 0;
uint32_t lastEncoder2 = 0;

// The encoders initialized flag
bool encodersInitialized = false;

// The md25 pointer
MD25 * md25Ptr;

// The odometry pointer
Odometry * odometryPtr;

// The pointer to odometry message publisher
ros::Publisher * odometryMessagePublisherPtr;

// The diagnostics message publisher pointer
ros::Publisher * diagnosticsMessagePublisherPtr;

// The odometry message
nav_msgs::Odometry odometryMessage;

void publishDiagnostics(uint8_t level, std::string message) {

	// Create diagnostics message
	diagnostic_msgs::DiagnosticArray diagnosticsMessage;
	diagnosticsMessage.header.stamp = ros::Time::now();
	diagnosticsMessage.status.resize(1);
	diagnosticsMessage.status[0].level = level;
	diagnosticsMessage.status[0].name = "MD25";
	diagnosticsMessage.status[0].message = message.c_str();
	diagnosticsMessage.status[0].hardware_id = "md25";

	// Read md25 diagnostics
	uint8_t version, volts, current1, current2;
	uint8_t result = (md25Ptr->getVersion(& version) == MD25_RESPONSE_OK &&
			md25Ptr->getVI(& volts, & current1, & current2) == MD25_RESPONSE_OK)
			? MD25_RESPONSE_OK : MD25_RESPONSE_ERROR;

	if (result == MD25_RESPONSE_OK) {

		// Get values as char array
		char versionChars[1];
		sprintf(versionChars, "%d", version);
		char voltsChars[1];
		sprintf(voltsChars, "%d", volts);
		char current1Chars[1];
		sprintf(current1Chars, "%d", current1);
		char current2Chars[1];
		sprintf(current2Chars, "%d", current2);

		diagnosticsMessage.status[0].values.resize(4);
		diagnosticsMessage.status[0].values[0].key = "Version";
		diagnosticsMessage.status[0].values[0].value = versionChars;
		diagnosticsMessage.status[0].values[1].key = "Volts";
		diagnosticsMessage.status[0].values[1].value = voltsChars;
		diagnosticsMessage.status[0].values[2].key = "Motor 1 current";
		diagnosticsMessage.status[0].values[2].value = current1Chars;
		diagnosticsMessage.status[0].values[3].key = "Motor 2 current";
		diagnosticsMessage.status[0].values[3].value = current2Chars;

	} else {

		// Log
    		ROS_WARN("Error reading diagnostics data: %d", result);

		// No values
		diagnosticsMessage.status[0].values.resize(0);

	}

	// Publish diagnostics message
	diagnosticsMessagePublisherPtr->publish(diagnosticsMessage);

}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmdVelMessage) {

	// Calculate cmd_speeds
	uint8_t speed1 = 128 + (cmdVelMessage->linear.x - cmdVelMessage->angular.z * wheelBase / 2) * speedSensitivity1 / wheelDiameter1;
	uint8_t speed2 = 128 + (cmdVelMessage->linear.x + cmdVelMessage->angular.z * wheelBase / 2) * speedSensitivity2 / wheelDiameter2;

	if (speed1 == 128 && speed2 == 128 && (cmdVelMessage->linear.x != 0 || cmdVelMessage->angular.z != 0)) {

		// Log
		ROS_WARN("Received velocities are too low for current MD25 configuration (%g, %g), check navigation stack setup for minimum speeds", cmdVelMessage->linear.x, cmdVelMessage->angular.z);

	}

	// Set sppeds on MD25
	uint8_t result = (md25Ptr->setSpeed1(speed1) == MD25_RESPONSE_OK &&
			md25Ptr->setSpeed2(speed2) == MD25_RESPONSE_OK) ? MD25_RESPONSE_OK : MD25_RESPONSE_ERROR;

	if (result != MD25_RESPONSE_OK) {

		// Log
		ROS_ERROR("Failed to set speed on MD25");

	}

}

void updateOdometry() {

	// Get now
	ros::Time now = ros::Time::now();

	// Read encoders
	uint32_t encoder1, encoder2;
	uint8_t result = md25Ptr->getEncoders(&encoder1, &encoder2);

	if (result != MD25_RESPONSE_OK) {

		// Log
		ROS_WARN("Error reading encoders: %d", result);

		// Publish diagnostics
		publishDiagnostics(STATUS_WARN, "Error reading encoders");

		return;

	}

	if (!encodersInitialized) {

		// Use encoders values to initialize odometry
		lastEncoder1 = encoder1;
		lastEncoder2 = encoder2;

		// Set encoders initialized to true
		encodersInitialized = true;

		// Log
		ROS_INFO("received first encoders values");

		return;

	}

	// Calculate delta encoders
	int32_t deltaEncoder1 = encoder1 - lastEncoder1;
	int32_t deltaEncoder2 = encoder2 - lastEncoder2;

	// Skip position update if both encoders are unchanged
	if (deltaEncoder1 != 0 || deltaEncoder2 != 0) {

		// Calculate delta s for each wheel
		double deltaS1 = deltaEncoder1 * encoderSensitivity1 * wheelDiameter1;
		double deltaS2 = deltaEncoder2 * encoderSensitivity2 * wheelDiameter2;

		// Update odometry
		odometryPtr->update(deltaS1, deltaS2, now.toSec());

		// Update encoders
		lastEncoder1 = encoder1;
		lastEncoder2 = encoder2;

	} else {

		// Update odometry time only
		odometryPtr->update(0, 0, now.toSec());

	}

	// Get linear and angular position
	tf::Vector3 linPos, angPos;
	odometryPtr->getPosition(linPos, angPos);

	// Get linear and angular velocity
	tf::Vector3 linVel, angVel;
	odometryPtr->getVelocity(linVel, angVel);

	// Get covariance
	tf::Matrix3x3 covariance;
	odometryPtr->getCovariance(covariance);

	// Create odometry quaternion from th
	geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(angPos.z());

	// Update odometry message
	odometryMessage.header.stamp = now;
	odometryMessage.pose.pose.position.x = linPos.x();
	odometryMessage.pose.pose.position.y = linPos.y();
	odometryMessage.pose.covariance[0] = covariance[0].x();
	odometryMessage.pose.covariance[1] = covariance[0].y();
	odometryMessage.pose.covariance[5] = covariance[0].z();
	odometryMessage.pose.covariance[6] = covariance[1].x();
	odometryMessage.pose.covariance[7] = covariance[1].y();
	odometryMessage.pose.covariance[11] = covariance[1].z();
	odometryMessage.pose.covariance[30] = covariance[2].x();
	odometryMessage.pose.covariance[31] = covariance[2].y();
	odometryMessage.pose.covariance[35] = covariance[2].z();
	odometryMessage.pose.pose.orientation = odometryQuaternion;
	odometryMessage.twist.twist.linear.x = linVel.x();
	odometryMessage.twist.twist.angular.z = angVel.z();

	// Publish odometry message
	odometryMessagePublisherPtr->publish(odometryMessage);

}

int main(int argc, char** argv) {

	// Init ros
	ros::init(argc, argv, "odometry_node");

	// Get node handle
	ros::NodeHandle nodeHandle;

	// Get private node handle
	ros::NodeHandle privateNodeHandle("~");

	// Get serial parameters
	privateNodeHandle.param<std::string>("port", port, "/dev/ttymxc2");
	privateNodeHandle.param("baudrate", baudrate, 38400);
	privateNodeHandle.param("timeout", timeout, 250);

	// Get ros parameters
	privateNodeHandle.param<std::string>("base_frame", baseFrame, "base_link");
	privateNodeHandle.param<std::string>("odom_frame", odomFrame, "odom");

	// Get other parameters
	privateNodeHandle.param("odometry_frequency", odometryFrequency, 10.0);
	privateNodeHandle.param("diagnostics_frequency", diagnosticsFrequency, 1.0);
	privateNodeHandle.param("encoder_sensitivity1", encoderSensitivity1, 0.00872639);
	privateNodeHandle.param("encoder_sensitivity2", encoderSensitivity2, 0.00872639);
	privateNodeHandle.param("speed_sensitivity1", speedSensitivity1, 12.4620);
	privateNodeHandle.param("speed_sensitivity2", speedSensitivity2, 12.4620);
	privateNodeHandle.param("wheel_diameter1", wheelDiameter1, 0.1);
	privateNodeHandle.param("wheel_diameter2", wheelDiameter2, 0.1);
	privateNodeHandle.param("wheel_base", wheelBase, 0.3);
	privateNodeHandle.param("cov_k1", covK1, 0.001);
	privateNodeHandle.param("cov_k2", covK2, 0.001);
	privateNodeHandle.param("cov_radius_threshold", covRTres, 10.0);
	privateNodeHandle.param("yaw_speed_variance", yawSpeedVariance, 0.06);

	// Log
	ROS_INFO("broadcasting transformation %s -> %s", odomFrame.c_str(), baseFrame.c_str());
	ROS_INFO("odometry update frequency: %g Hz", odometryFrequency);
	ROS_INFO("diagnostics update frequency: %g Hz", diagnosticsFrequency);
	ROS_INFO("encoder sensitivity1: %g LSB / rad", encoderSensitivity1);
	ROS_INFO("encoder sensitivity2: %g LSB / rad", encoderSensitivity2);
	ROS_INFO("speed sensitivity1: %g LSB / rad", speedSensitivity1);
	ROS_INFO("speed sensitivity2: %g LSB / rad", speedSensitivity2);
	ROS_INFO("wheel diameter1: %g m", wheelDiameter1);
	ROS_INFO("wheel diameter2: %g m", wheelDiameter2);
	ROS_INFO("wheel base: %g m", wheelBase);
	ROS_INFO("covariance constant 1: %g", covK1);
	ROS_INFO("covariance constant 2: %g", covK2);
	ROS_INFO("covariance radius threshold: %g m", covRTres);
	ROS_INFO("yaw speed variance: %g", yawSpeedVariance);

	// Create odometry
	Odometry odometry(wheelBase, covK1, covK2, covRTres);
	odometryPtr = &odometry;

	// Create odometry message publisher
	ros::Publisher odometryMessagePublisher = nodeHandle.advertise<nav_msgs::Odometry>("/odom", 20);
	odometryMessagePublisherPtr = & odometryMessagePublisher;

	// Create disgnostics message publisher
	ros::Publisher diagnosticsMessagePublisher = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 20, true);
	diagnosticsMessagePublisherPtr = & diagnosticsMessagePublisher;

	// Init odometry message
	odometryMessage.header.frame_id = odomFrame;
	odometryMessage.child_frame_id = baseFrame;
	odometryMessage.pose.pose.position.z = 0.0;
	odometryMessage.twist.covariance[35] = pow(yawSpeedVariance, 2);

	// Create cmd_vel message subscriber
	ros::Subscriber cmdVelMessageSubscriber = nodeHandle.subscribe("/cmd_vel", 20, cmdVelCallback);

	// Log
	ROS_INFO("Connecting to: %s port with baudrate: %d ...", port.c_str(), baudrate);

	// Create md25 and set the pointer
	MD25 md25(port, baudrate, timeout);
	md25Ptr = & md25;

	uint8_t result;

	// Log
	ROS_INFO("Setting up MD25...");

	// Setting up md25
	result = (md25.init() == MD25_RESPONSE_OK &&
		md25.setAcceleration(1) == MD25_RESPONSE_OK &&
		md25.disableTimeout() == MD25_RESPONSE_OK &&
		md25.setMode(0) == MD25_RESPONSE_OK &&
		md25.setSpeed1(128) == MD25_RESPONSE_OK &&
		md25.setSpeed2(128) == MD25_RESPONSE_OK) ? MD25_RESPONSE_OK : MD25_RESPONSE_ERROR;

	if (result != MD25_RESPONSE_OK) {

		// Log
		ROS_FATAL("MD25 configuration failed");

		// Publish diagnostics
		publishDiagnostics(STATUS_ERROR, "Configuration failed");

		return -1;

	}

	// Define odometry rate and diagnostic duration
	ros::Rate odometryRate(odometryFrequency);
	ros::Duration diagnosticsDuration(1 / diagnosticsFrequency);
	ros::Time lastDiagnosticsUpdateTime = ros::Time::now();

	while (ros::ok()) {

		// Update odometry
		updateOdometry();

		// Get now
		ros::Time now = ros::Time::now();

		if (now - lastDiagnosticsUpdateTime > diagnosticsDuration) {

			// Publish diagnostics
			publishDiagnostics(STATUS_OK, "OK");

			// Reset last diagnostics update time
			lastDiagnosticsUpdateTime = now;

		}

		// Spin
		ros::spinOnce();

		// Sleep
		odometryRate.sleep();

	}

	// Dispose
	result = md25.dispose();

	if (result != MD25_RESPONSE_OK) {

		// Log
		ROS_ERROR("Failed to dispose MD25");

	}

	return 0;

}
