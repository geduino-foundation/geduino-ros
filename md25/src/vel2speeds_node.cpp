/*
 vel2speeds_node.cpp

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
 vel2speeds_node is a ROS node implementation used as bridge between ROS navigation stack
 and and an MD25 node that receive speeds commands. It subscribe velocity command from 
 navigation stack and publish speeds command to MD25 node.
 
 Subscribes:
 - /cmd_vel (geometry_msgs/Twist): the velocity command.
 
 Publish:
 - cmd_speeds (md25_msgs/Speeds): the speeds command.

 Parameters:
 - cmd_vel_topic: the subscribed velocity command topic name (default: cmd_vel);
 - cmd_speeds_topic: the published speeds command topic name (default: cmd_speeds);
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
#include <md25_msgs/Speeds.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

// The odometry parameters
double speedSensitivity1;
double speedSensitivity2;
double wheelDiameter1;
double wheelDiameter2;
double wheelBase;

// The pointer to cmd speed message publisher
ros::Publisher * cmdSpeedsMessagePublisherPtr;

// The cmd_speeds message
md25_msgs::Speeds cmd_speeds;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmdVelMessage) {

  // Calculate cmd_speeds
  cmd_speeds.speed1 = (cmdVelMessage->linear.x - cmdVelMessage->angular.z * wheelBase / 2) * speedSensitivity1 / wheelDiameter1;
  cmd_speeds.speed2 = (cmdVelMessage->linear.x + cmdVelMessage->angular.z * wheelBase / 2) * speedSensitivity2 / wheelDiameter2;

  if (cmd_speeds.speed1 == 0 && cmd_speeds.speed2 == 0 && (cmdVelMessage->linear.x != 0 || cmdVelMessage->angular.z != 0)) {

     // Log
     ROS_WARN("received velocities are too low for current MD25 configuration (%g, &g), check navigation stack setup for minimum speeds", cmdVelMessage->linear.x, cmdVelMessage->angular.z);

  }

  // Publish cmd_speeds message
  cmdSpeedsMessagePublisherPtr->publish(cmd_speeds);

}

int main(int argc, char** argv) {

  // Init ros
  ros::init(argc, argv, "vel2speeds_node");

  // Get node handle
  ros::NodeHandle nodeHandle;

  // Get private node handle
  ros::NodeHandle privateNodeHandle("~");

  // Get odometry parameters
  privateNodeHandle.param("speed_sensitivity1", speedSensitivity1, 12.4620);
  privateNodeHandle.param("speed_sensitivity2", speedSensitivity2, 12.4620);
  privateNodeHandle.param("wheel_diameter1", wheelDiameter1, 0.1);
  privateNodeHandle.param("wheel_diameter2", wheelDiameter2, 0.1);
  privateNodeHandle.param("wheel_base", wheelBase, 0.3);

  // Log
  ROS_INFO("speed sensitivity1: %g LSB / rad", speedSensitivity1);
  ROS_INFO("speed sensitivity2: %g LSB / rad", speedSensitivity2);
  ROS_INFO("wheel diameter1: %g m", wheelDiameter1);
  ROS_INFO("wheel diameter2: %g m", wheelDiameter2);
  ROS_INFO("wheel base: %g m", wheelBase);

  // Create cmd_vel message subscriber
  ros::Subscriber cmdVelMessageSubscriber = nodeHandle.subscribe("/cmd_vel", 20, cmdVelCallback);

  // Create cmd_speeds message publisher
  ros::Publisher cmdSpeedsMessagePublisher = nodeHandle.advertise<md25_msgs::Speeds>("cmd_speeds", 20);
  cmdSpeedsMessagePublisherPtr = & cmdSpeedsMessagePublisher;

  // Spin
  ros::spin();

  return 0;

}
