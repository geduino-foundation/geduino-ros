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
 In order to refine odometry IMU data can be fused to improve reliability.

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
 - pos_covariance_diagonal: the diagonal of the position covariance matrix (a 6 element
   vector of type double);
 - vel_covariance_diagonal: the diagonal of the velocity covariance matrix (a 6 element
   vector of type double);
 - rolling_window_size: the size of rolling windows used to apply median filter in velocity
   computation (default: 3);
 - publish_odom_transformation: true if odom frame -> base frame transformation must be
   published by this node, false otherwise (default: true)
 - enable_complementary_filter: true to enable complementary filter to fuse odometry data
   with IMU data, false otherwise (default: false)
 - position_filter_tau: the position filter parameter to fuse encoders and IMU infos. Set it
   to zero to disable filter (default: 0.075)
 - velocity_filter_tau: the velocity filter parameter to fuse encoders and IMU infos. Set it
   to zero to disable filter (default: 0.2)
 - imu_cache_size: the IMU message cache size (default: 10)
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
std::vector<double> posCovarianceDiagonal;
std::vector<double> velCovarianceDiagonal;
int rollingWindowSize;
bool publishOdomTransformation;
bool enableComplementaryFilter;
double positionFilterTau;
double velocityFilterTau;
int imuMessageCacheSize;

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

// The pointer to imu message cache
message_filters::Cache<sensor_msgs::Imu> * imuMessageCachePtr;

// The pointer tp odometry transform broadcaster
tf::TransformBroadcaster * odometryTransformBroadcasterPtr;

// The pointer to transform listener
tf::TransformListener * transformListenerPtr;

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

    // Convert volts to float
    float voltsFloat = 0.1 * volts;

    // Covert current to float
    float current1Float = 0.1 * current1;
    float current2Float = 0.1 * current2;

	if (result == MD25_RESPONSE_OK) {

		// Get values as char array
		char versionChars[3];
		sprintf(versionChars, "%d", version);
		char voltsChars[6];
        sprintf(voltsChars, "%g V", voltsFloat);
		char current1Chars[6];
        sprintf(current1Chars, "%g A", current1Float);
		char current2Chars[6];
        sprintf(current2Chars, "%g A", current2Float);

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
        ROS_WARN("Received velocities are too low for current MD25 configuration (%g, %g), check navigation stack setup for minimum speeds",
                 cmdVelMessage->linear.x, cmdVelMessage->angular.z);

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

    // Get angular velocity in base frame
    geometry_msgs::Vector3Stamped angularVelocityInBaseFrame;

    if (enableComplementaryFilter) {

        // Get imu message
        sensor_msgs::ImuConstPtr imuMessagePtr = imuMessageCachePtr->getElemBeforeTime(now);

        if (imuMessagePtr == NULL) {

            // Log
            ROS_ERROR_THROTTLE(1.0, "Imu message not received at %f", now.toSec());

            return;

        }

        // Get angular velocity in IMU frame
        geometry_msgs::Vector3Stamped angularVelocityInImuFrame;
        angularVelocityInImuFrame.header = imuMessagePtr->header;
        angularVelocityInImuFrame.vector = imuMessagePtr->angular_velocity;

        std::string errorMsg;

        if (transformListenerPtr->canTransform(baseFrame, imuMessagePtr->header.frame_id, now, & errorMsg)) {

            // Transform angular velocity in base frame
            transformListenerPtr->transformVector(baseFrame, angularVelocityInImuFrame, angularVelocityInBaseFrame);

        } else {

            // Log
            ROS_ERROR_THROTTLE(1.0, "Cannot transform from %s -> %s at %f: %s", imuMessagePtr->header.frame_id.c_str(),
                               baseFrame.c_str(), now.toSec(), errorMsg.c_str());

            return;

        }

    }

	// Calculate delta encoders
	int32_t deltaEncoder1 = encoder1 - lastEncoder1;
    int32_t deltaEncoder2 = encoder2 - lastEncoder2;

    // Calculate delta s for each wheel
    double deltaS1 = deltaEncoder1 * encoderSensitivity1 * wheelDiameter1;
    double deltaS2 = deltaEncoder2 * encoderSensitivity2 * wheelDiameter2;

    // Update odometry
    odometryPtr->update(deltaS1, deltaS2, angularVelocityInBaseFrame.vector.z, now.toSec());

    // Update encoders
    lastEncoder1 = encoder1;
    lastEncoder2 = encoder2;

    // Get position
    Vector3 pos;
    odometryPtr->getFilteredPosition(pos);

    // Get velocity
    Vector3 vel;
    odometryPtr->getFilteredVelocity(vel);

	// Create odometry quaternion from th
    geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(pos(2));

    // Publish odometry message
    nav_msgs::Odometry odometryMessage;
    odometryMessage.header.stamp = now;
    odometryMessage.header.frame_id = odomFrame;
    odometryMessage.child_frame_id = baseFrame;
    odometryMessage.pose.pose.position.x = pos(0);
    odometryMessage.pose.pose.position.y = pos(1);
    odometryMessage.pose.pose.position.z = 0.0;
    odometryMessage.pose.pose.orientation = odometryQuaternion;
    odometryMessage.twist.twist.linear.x = vel(0);
    odometryMessage.twist.twist.linear.y = vel(1);
    odometryMessage.twist.twist.angular.z = vel(2);
    odometryMessage.pose.covariance[0] = posCovarianceDiagonal[0];
    odometryMessage.pose.covariance[7] = posCovarianceDiagonal[1];
    odometryMessage.pose.covariance[14] = posCovarianceDiagonal[2];
    odometryMessage.pose.covariance[21] = posCovarianceDiagonal[3];
    odometryMessage.pose.covariance[28] = posCovarianceDiagonal[4];
    odometryMessage.pose.covariance[35] = posCovarianceDiagonal[5];
    odometryMessage.twist.covariance[0] = velCovarianceDiagonal[0];
    odometryMessage.twist.covariance[7] = velCovarianceDiagonal[1];
    odometryMessage.twist.covariance[14] = velCovarianceDiagonal[2];
    odometryMessage.twist.covariance[21] = velCovarianceDiagonal[3];
    odometryMessage.twist.covariance[28] = velCovarianceDiagonal[4];
    odometryMessage.twist.covariance[35] = velCovarianceDiagonal[5];

	// Publish odometry message
	odometryMessagePublisherPtr->publish(odometryMessage);

    if (publishOdomTransformation) {

        // Send odometry transformation
        geometry_msgs::TransformStamped odometryTransformation;
        odometryTransformation.header.stamp = now;
        odometryTransformation.header.frame_id = odomFrame;
        odometryTransformation.child_frame_id = baseFrame;
        odometryTransformation.transform.translation.z = 0.0;
        odometryTransformation.transform.translation.x = pos(0);
        odometryTransformation.transform.translation.y = pos(1);
        odometryTransformation.transform.rotation = odometryQuaternion;

        // Broadcast odometry transformation
        odometryTransformBroadcasterPtr->sendTransform(odometryTransformation);

    }

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
    privateNodeHandle.getParam("pos_covariance_diagonal", posCovarianceDiagonal);
    privateNodeHandle.getParam("vel_covariance_diagonal", velCovarianceDiagonal);
    privateNodeHandle.param("rolling_window_size", rollingWindowSize, 3);
    privateNodeHandle.param("publish_odom_transformation", publishOdomTransformation, true);
    privateNodeHandle.param("enable_complementary_filter", enableComplementaryFilter, false);
    privateNodeHandle.param("position_filter_tau", positionFilterTau, 0.075);
    privateNodeHandle.param("velocity_filter_tau", velocityFilterTau, 0.2);
    privateNodeHandle.param("imu_message_cache_size", imuMessageCacheSize, 10);

    if (posCovarianceDiagonal.size() != 6) {

        // Log
        ROS_WARN("pos_covariance_diagonal size must be 6, actual: %lu. Use zeroes.", posCovarianceDiagonal.size());

        // Assign a six sized vector fill with zeroes
        posCovarianceDiagonal.assign(6, 0.0f);

    }

    if (velCovarianceDiagonal.size() != 6) {

        // Log
        ROS_WARN("vel_covariance_diagonal size must be 6, actual: %lu. Use zeroes.", velCovarianceDiagonal.size());

        // Assign a six sized vector fill with zeroes
        velCovarianceDiagonal.assign(6, 0.0f);

    }

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
    ROS_INFO("rolling window size: %d", rollingWindowSize);
    ROS_INFO("publish odom transformation: %d", publishOdomTransformation);
    ROS_INFO("enable complementary filter: %d", enableComplementaryFilter);
    ROS_INFO("position filter tau: %g", positionFilterTau);
    ROS_INFO("velocity filter tau: %g", velocityFilterTau);
    ROS_INFO("imu message cache size: %d", imuMessageCacheSize);

	// Create odometry
    Odometry odometry(wheelBase, rollingWindowSize, enableComplementaryFilter ? positionFilterTau : 0,
                      enableComplementaryFilter ? velocityFilterTau : 0);
	odometryPtr = &odometry;

	// Create odometry message publisher
	ros::Publisher odometryMessagePublisher = nodeHandle.advertise<nav_msgs::Odometry>("/odom", 20);
	odometryMessagePublisherPtr = & odometryMessagePublisher;

	// Create disgnostics message publisher
	ros::Publisher diagnosticsMessagePublisher = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 20, true);
	diagnosticsMessagePublisherPtr = & diagnosticsMessagePublisher;

    // The odometry transform broadcaster
    tf::TransformBroadcaster odometryTransformBroadcaster;
    odometryTransformBroadcasterPtr = & odometryTransformBroadcaster;

	// Create cmd_vel message subscriber
    nodeHandle.subscribe("/cmd_vel", 20, cmdVelCallback);

    if (enableComplementaryFilter) {

        // The transform listener
        tf::TransformListener transformListener(nodeHandle);
        transformListenerPtr = & transformListener;

        // Create imu message subscriber and cache
        message_filters::Subscriber<sensor_msgs::Imu> imuMessageSubscriber(nodeHandle, "/imu", 1);
        message_filters::Cache<sensor_msgs::Imu> imuMessageCache(imuMessageSubscriber, imuMessageCacheSize);
        imuMessageCachePtr = & imuMessageCache;

    }

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
