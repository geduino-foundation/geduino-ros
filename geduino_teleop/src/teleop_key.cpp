/*
 teleop_key.cpp

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
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

double linearXspeed;
double angularZspeed;

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {

  // Give back contro to terminall window
  tcsetattr(kfd, TCSANOW, &cooked);

  // SHutdown ROS
  ros::shutdown();

  // Exit
  exit(0);

}

int main(int argc, char** argv) {

  // Init teleop_key
  ros::init(argc, argv, "teleop_key");

  // Create private ROS node handle
  ros::NodeHandle privateNodeHandle("~");
  privateNodeHandle.param("linear_x_speed", linearXspeed, 0.1);
  privateNodeHandle.param("angular_z_speed", angularZspeed, 0.52);

  // Log
  ROS_INFO("linear x speed: %g m/s", linearXspeed);
  ROS_INFO("angular z speed: %g rad/s", angularZspeed);

  // Create ROS node handle
  ros::NodeHandle nodeHandle;

  // Create the_cmd vel publisher
  ros::Publisher cmdVelPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // Attach quit method to interrupt signal
  signal(SIGINT, quit);

  // Get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  // Show info
  puts("---------------------------");
  puts("Use arrow keys to move Geduino.");
  puts("Use q key to stop Geduino.");

  // Set tangent and angular speeds
  float tangentSpeed = 0;
  float angularSpeed = 0;

  // Init modified and stopped
  bool modified = false;
  bool stopped = false;

  while (!stopped) {

    char c;

    // Read char
    if (read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

	// Execute command
    switch(c) {

      case KEYCODE_U:
	 puts("forward...");
         tangentSpeed = linearXspeed;
         angularSpeed = 0;
         modified = true;
         break;

      case KEYCODE_D:
	 puts("back...");
         tangentSpeed = -linearXspeed;
         angularSpeed = 0;
         modified = true;
         break;

      case KEYCODE_L:
	 puts("turn left...");
         angularSpeed = angularZspeed;
         modified = true;
         break;

      case KEYCODE_R:
	 puts("turn right...");
         angularSpeed = -angularZspeed;
         modified = true;
         break;

      case KEYCODE_Q:
	 puts("stop...");
         tangentSpeed = 0;
         angularSpeed = 0;
         modified = true;
         break;

    }

    if (modified) {

      // Create cmd_vel message
      geometry_msgs::Twist cmdVel;
      cmdVel.linear.x = tangentSpeed;
      cmdVel.angular.z = angularSpeed;

      // Publish cmd_vel message
      cmdVelPublisher.publish(cmdVel);

      // Spin once
      ros::spinOnce();

      modified = false;

    }

  }

  return (0);

}
