/*
 odometry_node.cpp

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
 odometry_node is a ROS node implementation used as bridge between ROS navigation stack
 and and an MD25 node that is able to publish encoders data and receive speeds commands.
 This node subscribe encoders topic from MD25 and publish odometry transformation and
 topic to the navigation stack. Furthermore it subscribe velocity command from navigation
 stack and publish speeds command to MD25 node.

 The covariance model used get from work of Linsday KLEEMAN of Monash University (technical
 report MECSE-95-1-1995).
 This model assume that variance of travelled space for a wheel is proportial to travelled 
 space by a constant.
 
 Subscribes:
 - md25/encoders (md25_msgs/StampedEncoders): the encoder values;
 - cmd_vel (geometry_msgs/Twist): the velocity command.
 
 Publish:
 - tf (tf/tfMessage): the odom -> base_link transformation;
 - odom (nav_msgs/Odometry): the odometry topic;
 - cmd_speeds (md25_msgs/Speeds): the speeds command.

 Parameters:
 - encoders_topic, the subscribed encoders topic name (default: md25/encoders);
 - base_frame, the frame attached to robot base, i.e. broadcasted transformation child
   frame (default: base_link);
 - odom_frame, : odometry frame, i.e. broadcasted transformation frame (default: odom);
 - odom_topic: the published odometry topic name (default: odom);
 - cmd_vel_topic: the subscribed velocity command topic name (default: cmd_vel);
 - cmd_speeds_topic: the published speeds command topic name (default: cmd_speeds);
 - encoderSensitivity1: the encoder 1 sensitivity in LSB / rad. It represent the inverse
   of wheel shaft rotation angle for unitary encoder increment. (default: 0.00872639,
   valid using EMG30 motor);
 - encoderSensitivity2: the encoder 2 sensitivity in LSB / rad. It represent the inverse
   of wheel shaft rotation angle for unitary encoder increment. (default: 0.00872639,
   valid using EMG30 motor);
 - speedSensitivity1: the speed 1 sensitivity in LSB / (rad/s). It represent the inverse
   of wheel shaft rotation speed for unitary speed input. (default: 12.4620, valid using
   EMG30 motor);
 - speedSensitivity2: the speed 2 sensitivity in LSB / (rad/s). It represent the inverse
   of wheel shaft rotation speed for unitary speed input. (default: 12.4620, valid using
   EMG30 motor);
 - wheel_diameter1: the wheel 1 diameter in m (default: 0.1);
 - wheel_diameter2: the wheel 2 diameter in m (default: 0.1);
 - wheel_base: the wheel base in m (default: 0.3);
 - cov_K1: the constant of covariance model for wheel 1;
 - cov_K2: the constant of covariance model for wheel 2;
 - cov_radius_threshold: the radius threshold to distinguish between straight andarc path.
 */
 
#include <ros/ros.h>
#include <md25_msgs/StampedEncoders.h>
#include <md25_msgs/Speeds.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <odometry.h>

// The ros parameters
std::string encoderTopic;
std::string baseFrame;
std::string odomFrame;
std::string odomTopic;
std::string cmdVelTopic;
std::string cmdSpeedsTopic;

// The odometry parameters
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

// The last encoder values
uint32_t lastEncoder1 = 0;
uint32_t lastEncoder2 = 0;

// The encoders initialized flag
bool encodersInitialized = false;

// The odometry
Odometry * odometryPtr;

// The pointer to cmd speed message publisher
ros::Publisher * cmdSpeedsMessagePublisherPtr;

// The cmd_speeds message
md25_msgs::Speeds cmd_speeds;

// The pointer to odometry message publisher
ros::Publisher * odometryMessagePublisherPtr;

// The pointer tp odometry transform broadcaster
tf::TransformBroadcaster * odometryTransformBroadcasterPtr;

// The odometry transformation
geometry_msgs::TransformStamped odometryTransformation;

// The odometry message
nav_msgs::Odometry odometryMessage;

void encodersCallback(const md25_msgs::StampedEncoders::ConstPtr & encodersMessage) {

  if (!encodersInitialized) {

    // Use encoders values to initialize odometry
    lastEncoder1 = encodersMessage->encoders.encoder1;
    lastEncoder2 = encodersMessage->encoders.encoder2;

    // Set encoders initialized to true
    encodersInitialized = true;

    // Log
    ROS_INFO("received first encoders values");

    return;

  }

  // Calculate delta encoders
  int32_t deltaEncoder1 = encodersMessage->encoders.encoder1 - lastEncoder1;
  int32_t deltaEncoder2 = encodersMessage->encoders.encoder2 - lastEncoder2;

  // Skip position update if both encoders are unchanged
  if (deltaEncoder1 != 0 || deltaEncoder2 != 0) {

    // Calculate delta s for each wheel
    double deltaS1 = deltaEncoder1 * encoderSensitivity1 * wheelDiameter1;
    double deltaS2 = deltaEncoder2 * encoderSensitivity2 * wheelDiameter2;

    // Update odometry
    odometryPtr->update(deltaS1, deltaS2);

    // Update encoders
    lastEncoder1 = encodersMessage->encoders.encoder1;
    lastEncoder2 = encodersMessage->encoders.encoder2;

  }

  // Get position
  tf::Vector3 position;
  odometryPtr->getPosition(position);

  // Get covariance
  tf::Matrix3x3 covariance;
  odometryPtr->getCovariance(covariance);

  // Create odometry quaternion from th
  geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(position.z());

  // Update odometry transformation
  odometryMessage.header.stamp = encodersMessage->header.stamp;
  odometryTransformation.transform.translation.x = position.x();
  odometryTransformation.transform.translation.y = position.y();
  odometryTransformation.transform.rotation = odometryQuaternion;

  // Update odometry message
  odometryTransformation.header.stamp = encodersMessage->header.stamp;
  odometryMessage.pose.pose.position.x = position.x();
  odometryMessage.pose.pose.position.y = position.y();
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

  // Broadcast odometry transformation
  odometryTransformBroadcasterPtr->sendTransform(odometryTransformation);

  // Publish odometry message
  odometryMessagePublisherPtr->publish(odometryMessage);

}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmdVelMessage) {

  // Calculate cmd_speeds
  cmd_speeds.speed1 = (cmdVelMessage->linear.x - cmdVelMessage->angular.z * wheelBase / 2) * speedSensitivity1 / wheelDiameter1;
  cmd_speeds.speed2 = (cmdVelMessage->linear.x + cmdVelMessage->angular.z * wheelBase / 2) * speedSensitivity2 / wheelDiameter2;

  if (cmd_speeds.speed1 == 0 && cmd_speeds.speed2 == 0 && (cmdVelMessage->linear.x != 0 || cmdVelMessage->angular.z != 0)) {

     // Log
     ROS_WARN("received velocities are too low for current MD25 configuration (%g, &g), check navigation stack setup for minimum speeds", cmdVelMessage->linear.x, cmdVelMessage->angular.z);

  }

  // Calculate actual_vel
  odometryMessage.twist.twist.linear.x = (cmd_speeds.speed1 * wheelDiameter1 / speedSensitivity1 + cmd_speeds.speed2 * wheelDiameter2 / speedSensitivity2) / 2;
  odometryMessage.twist.twist.angular.z = (cmd_speeds.speed1 * wheelDiameter1 / speedSensitivity1 - cmd_speeds.speed2 * wheelDiameter2 / speedSensitivity2) / wheelBase;

  // Publish cmd_speeds message
  cmdSpeedsMessagePublisherPtr->publish(cmd_speeds);

}

int main(int argc, char** argv) {

  // Init ros
  ros::init(argc, argv, "odometry_node");

  // Get node handle
  ros::NodeHandle nodeHandle;

  // Get private node handle
  ros::NodeHandle privateNodeHandle("~");

  // Get ros parameters
  privateNodeHandle.param<std::string>("odometry_node/encoders_topic", encoderTopic, "md25/encoders");
  privateNodeHandle.param<std::string>("base_frame", baseFrame, "base_link");
  privateNodeHandle.param<std::string>("odom_frame", odomFrame, "odom");
  privateNodeHandle.param<std::string>("odom_topic", odomTopic, "odom");
  privateNodeHandle.param<std::string>("cmd_vel_topic", cmdVelTopic, "cmd_vel");
  privateNodeHandle.param<std::string>("cmd_speeds_topic", cmdSpeedsTopic, "cmd_speeds");

  // Get odometry parameters
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

  // Log
  ROS_INFO("subscribed encoders topic: %s", encoderTopic.c_str());
  ROS_INFO("broadcasting transformation %s -> %s", odomFrame.c_str(), baseFrame.c_str());
  ROS_INFO("published odom topic: %s", odomTopic.c_str());
  ROS_INFO("subscribed cmd_vel topic: %s", cmdVelTopic.c_str());
  ROS_INFO("published cmd_speeds topic: %s", cmdSpeedsTopic.c_str());
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

  // Create odometry
  Odometry odometry(wheelBase, covK1, covK2, covRTres);
  odometryPtr = &odometry;

  // Create encoders message subscriber
  ros::Subscriber encodersMessageSubscriber = nodeHandle.subscribe(encoderTopic, 20, encodersCallback);

  // Create odometry message publisher
  ros::Publisher odometryMessagePublisher = nodeHandle.advertise<nav_msgs::Odometry>(odomTopic, 20);
  odometryMessagePublisherPtr = & odometryMessagePublisher;

  // The odometry transform broadcaster
  tf::TransformBroadcaster odometryTransformBroadcaster;
  odometryTransformBroadcasterPtr = & odometryTransformBroadcaster;

  // Create cmd_vel message subscriber
  ros::Subscriber cmdVelMessageSubscriber = nodeHandle.subscribe(cmdVelTopic, 20, cmdVelCallback);

  // Create cmd_speeds message publisher
  ros::Publisher cmdSpeedsMessagePublisher = nodeHandle.advertise<md25_msgs::Speeds>(cmdSpeedsTopic, 20);

  // Init odometry transformation
  odometryTransformation.header.frame_id = odomFrame;
  odometryTransformation.child_frame_id = baseFrame;
  odometryTransformation.transform.translation.z = 0.0;

  // Init odometry message
  odometryMessage.header.frame_id = odomFrame;
  odometryMessage.child_frame_id = baseFrame;
  odometryMessage.pose.pose.position.z = 0.0;

  // Spin
  ros::spin();

  return 0;

}
