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
 - frequency: the main loop frequency in Hz. In order to not loose odometry information
   this param should be set at value greater than published encoders topic frequency.
   (defauld: 20.0);
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
 - wheel_base: the wheel base in m (default: 0.3).
 */
 
#include <ros/ros.h>
#include <md25_msgs/StampedEncoders.h>
#include <md25_msgs/Speeds.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

// The ros parameters
std::string encoderTopic;
std::string baseFrame;
std::string odomFrame;
std::string odomTopic;
std::string cmdVelTopic;
std::string cmdSpeedsTopic;
double frequency;

// The odometry parameters
double encoderSensitivity1;
double encoderSensitivity2;
double speedSensitivity1;
double speedSensitivity2;
double wheelDiameter1;
double wheelDiameter2;
double wheelBase;

// The last encoder values
uint32_t lastEncoder1 = 0;
uint32_t lastEncoder2 = 0;

// The odometry position
double x = 0;
double y = 0;
double th = 0;

// The odometry transformation
geometry_msgs::TransformStamped odometryTransformation;

// The odometry message
nav_msgs::Odometry odometryMessage;

// The cmd_speeds message
md25_msgs::Speeds cmd_speeds;

// The speed updated flag
bool speedUpdated;

void encodersCallback(const md25_msgs::StampedEncoders::ConstPtr & encodersMessage) {

  // Calculate delta encoders
  int32_t deltaEncoder1 = encodersMessage->encoders.encoder1 - lastEncoder1;
  int32_t deltaEncoder2 = encodersMessage->encoders.encoder2 - lastEncoder2;

  // Skip position update if both encoders are unchanged
  if (deltaEncoder1 != 0 || deltaEncoder2 != 0) {

    // Calculate delta s for each wheel
    double deltaS1 = deltaEncoder1 * encoderSensitivity1 * wheelDiameter1;
    double deltaS2 = deltaEncoder2 * encoderSensitivity2 * wheelDiameter2;

    // Calculate delta s and delta theta
    double deltaS = (deltaS1 + deltaS2) / 2;
    double deltaTh = (deltaS2 - deltaS1) / wheelBase;

    // Calculate middle theta
    double middleTh = th + deltaTh / 2;

    // Update position
    x += deltaS * cos(middleTh);
    y += deltaS * sin(middleTh);
    th += deltaTh;

    // Update encoders
    lastEncoder1 = encodersMessage->encoders.encoder1;
    lastEncoder2 = encodersMessage->encoders.encoder2;

  }

  // Create odometry quaternion from th
  geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);

  // Update odometry transformation
  odometryTransformation.header.stamp = encodersMessage->header.stamp;
  odometryTransformation.transform.translation.x = x;
  odometryTransformation.transform.translation.y = y;
  odometryTransformation.transform.rotation = odometryQuaternion;

  // Update odometry message
  odometryMessage.header.stamp = encodersMessage->header.stamp;
  odometryMessage.pose.pose.position.x = x;
  odometryMessage.pose.pose.position.y = y;
  odometryMessage.pose.pose.orientation = odometryQuaternion;

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

  // Set speed updated
  speedUpdated = true;

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
  privateNodeHandle.param("frequency", frequency, 20.0);

  // Get odometry parameters
  privateNodeHandle.param("encoder_sensitivity1", encoderSensitivity1, 0.00872639);
  privateNodeHandle.param("encoder_sensitivity2", encoderSensitivity2, 0.00872639);
  privateNodeHandle.param("speed_sensitivity1", speedSensitivity1, 12.4620);
  privateNodeHandle.param("speed_sensitivity2", speedSensitivity2, 12.4620);
  privateNodeHandle.param("wheel_diameter1", wheelDiameter1, 0.1);
  privateNodeHandle.param("wheel_diameter2", wheelDiameter2, 0.1);
  privateNodeHandle.param("wheel_base", wheelBase, 0.3);

  // Log
  ROS_INFO("subscribed encoders topic: %s", encoderTopic.c_str());
  ROS_INFO("broadcasting transformation %s -> %s", odomFrame.c_str(), baseFrame.c_str());
  ROS_INFO("published odom topic: %s", odomTopic.c_str());
  ROS_INFO("subscribed cmd_vel topic: %s", cmdVelTopic.c_str());
  ROS_INFO("published cmd_speeds topic: %s", cmdSpeedsTopic.c_str());
  ROS_INFO("update frequency: %g Hz", frequency);
  ROS_INFO("encoder sensitivity1: %g LSB / rad", encoderSensitivity1);
  ROS_INFO("encoder sensitivity2: %g LSB / rad", encoderSensitivity2);
  ROS_INFO("speed sensitivity1: %g LSB / rad", speedSensitivity1);
  ROS_INFO("speed sensitivity2: %g LSB / rad", speedSensitivity2);
  ROS_INFO("wheel diameter1: %g m", wheelDiameter1);
  ROS_INFO("wheel diameter2: %g m", wheelDiameter2);
  ROS_INFO("wheel base: %g m", wheelBase);

  // Create encoders message subscriber
  ros::Subscriber encodersMessageSubscriber = nodeHandle.subscribe(encoderTopic, 20, encodersCallback);

  // Create odometry message publisher
  ros::Publisher odometryMessagePublisher = nodeHandle.advertise<nav_msgs::Odometry>(odomTopic, 20);

  // The odometry transform broadcaster
  tf::TransformBroadcaster odometryTransformBroadcaster;

 // Create cmd_vel message subscriber
  ros::Subscriber cmdVelMessageSubscriber = nodeHandle.subscribe(cmdVelTopic, 20, cmdVelCallback);

  // Create cmd_speeds message publisher
  ros::Publisher cmdSpeedsMessagePublisher = nodeHandle.advertise<md25_msgs::Speeds>(cmdSpeedsTopic, 20);

  // Create rate
  ros::Rate rate(frequency);

  // Init odometry transformation
  odometryTransformation.header.frame_id = odomFrame;
  odometryTransformation.child_frame_id = baseFrame;
  odometryTransformation.transform.translation.z = 0.0;

  // Init odometry message
  odometryMessage.header.frame_id = odomFrame;
  odometryMessage.child_frame_id = baseFrame;
  odometryMessage.pose.pose.position.z = 0.0;

  while (nodeHandle.ok()) {

     // Sleep
     rate.sleep();

     // Reset speed updated
     speedUpdated = false;

     // Spin, allow incoming topic to be received
     ros::spinOnce();

     // Broadcast odometry transformation
     odometryTransformBroadcaster.sendTransform(odometryTransformation);

     // Publish odometry message
     odometryMessagePublisher.publish(odometryMessage);

     if (speedUpdated) {

       // Publish cmd_speeds message
       cmdSpeedsMessagePublisher.publish(cmd_speeds);

     }
     
     // Spin, allow outcoming topic and transformation to be published
     ros::spinOnce();

  }

  return 0;

}
