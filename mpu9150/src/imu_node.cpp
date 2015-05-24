/*
 imu_node.cpp
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

#include <ros/ros.h>
#include <mpu9150_msgs/StampedMotion9.h>
#include <sensor_msgs/Imu.h>
#include <mpu9150.h>

// The ros parameters
std::string imuFrame;

// The mpu9150
MPU9150 mpu9150;

// The imu message publisher
ros::Publisher * imuMessagePublisher;

void motion9Callback(const mpu9150_msgs::StampedMotion9::ConstPtr & motion9Message) {

  // Create imu message
  sensor_msgs::Imu imuMessage;
  imuMessage.header.stamp = motion9Message->header.stamp;
  imuMessage.header.frame_id = imuFrame;

  // Get orientation
  imuMessage.orientation.x = motion9Message->motion9.orientation.x;
  imuMessage.orientation.y = motion9Message->motion9.orientation.y;
  imuMessage.orientation.z = motion9Message->motion9.orientation.z;
  imuMessage.orientation.w = motion9Message->motion9.orientation.w;

  // Compute linear acceleration
  int16_vector3 accelLSB;
  accelLSB.x = motion9Message->motion9.accel.x;
  accelLSB.y = motion9Message->motion9.accel.y;
  accelLSB.z = motion9Message->motion9.accel.z;
  double_vector3 accelMS2;
  mpu9150.accelLSB2MS2(&accelLSB, &accelMS2, motion9Message->motion9.full_scale_accel_range);
  imuMessage.linear_acceleration.x = accelMS2.x;
  imuMessage.linear_acceleration.y = accelMS2.y;
  imuMessage.linear_acceleration.z = accelMS2.z;

  // Compute angular velocity
  int16_vector3 gyroLSB;
  gyroLSB.x = motion9Message->motion9.gyro.x;
  gyroLSB.y = motion9Message->motion9.gyro.y;
  gyroLSB.z = motion9Message->motion9.gyro.z;
  double_vector3 gyroRS;
  mpu9150.gyroLSB2RS(&gyroLSB, &gyroRS, motion9Message->motion9.full_scale_gyro_range);
  imuMessage.angular_velocity.x = gyroRS.x;
  imuMessage.angular_velocity.y = gyroRS.y;
  imuMessage.angular_velocity.z = gyroRS.z;

  // Publish imu message
  imuMessagePublisher->publish(imuMessage);

}

int main(int argc, char** argv) {

  // Init ros
  ros::init(argc, argv, "imu_node");

  // Get node handle
  ros::NodeHandle nodeHandle;

  // Get private node handle
  ros::NodeHandle privateNodeHandle("~");

  // Get ros parameters
  privateNodeHandle.param<std::string>("imu_frame", imuFrame, "base_imu");

  // Log
  ROS_INFO("imu frame: %s", imuFrame.c_str());

  // Create motion9 message subscriber
  ros::Subscriber motion9MessageSubscriber = nodeHandle.subscribe("motion9", 20, motion9Callback);

  // Create imu message publisher
  ros::Publisher _imuMessagePublisher = nodeHandle.advertise<sensor_msgs::Imu>("/imu", 20);;
  imuMessagePublisher = &_imuMessagePublisher;

  // Spin
  ros::spin();

  return 0;

}
