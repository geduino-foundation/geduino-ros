/*
 imu_quaternon_to_euler.cpp

 Copyright (C) 2016 Alessandro Francescon
 
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

/*
 A debugger node to convert imu orientation from quaternion to euler angles.
 Subscribe:
 - /imu (sensor_msgs/Imu.h), the imu source
 Publish:
 - /imu_euler (geometry_message/Vector3.h), the euler angles
*/

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"

// The euler publisher pointer
ros::Publisher * eulerPublisherPtr;

void imuMessageCallback(const sensor_msgs::Imu::ConstPtr & imuMessage) {

	// Transform imu orientation quaternion to a tf::Quaterion
	tf::Quaternion quaternion;
	tf::quaternionMsgToTF(imuMessage->orientation, quaternion);

	// Create vector3 message for euler
	geometry_msgs::Vector3 euler;

	// Get euler roll, pitch and yaw
	tf::Matrix3x3(quaternion).getRPY(euler.x, euler.y, euler.z);

	// Publish euler
	eulerPublisherPtr->publish(euler);

}

int main(int argc, char **argv) {

	// Init node
	ros::init(argc, argv, "imu_quaternion_to_euuler");

	// The node handle
	ros::NodeHandle nodeHandle;

	// Create imu message subscriber
	ros::Subscriber imuSubscriber = nodeHandle.subscribe("/imu", 1000, imuMessageCallback);

	// Create euler message publisher
	ros::Publisher eulerPublisher = nodeHandle.advertise<geometry_msgs::Vector3>("/imu_euler", 1000);
	eulerPublisherPtr = & eulerPublisher;

	// Log
	ROS_INFO("waiting for imu message");

	// Spin
	ros::spin();

	return 0;

}
