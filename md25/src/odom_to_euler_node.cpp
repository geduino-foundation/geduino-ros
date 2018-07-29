/*
 odom_to_euler.cpp

 Copyright (C) 2018 Alessandro Francescon

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
#include <nav_msgs/Odometry.h>
#include <md25/StampedEuler.h>
#include <geometry_conversion.h>

// The pointer to euler message publisher
ros::Publisher * eulerMessagePublisherPtr;

void odomCallback(const nav_msgs::Odometry::ConstPtr & odomMessage) {

    // Convert to euler
    md25::StampedEuler stampedEuler;
    toEuler(odomMessage->pose.pose.orientation, & stampedEuler.euler);

    // Set header
    stampedEuler.header = odomMessage->header;

    // Publish stamped euler
    eulerMessagePublisherPtr->publish(stampedEuler);

}

int main(int argc, char** argv) {

    // Init ros
    ros::init(argc, argv, "quaternion_to_rpy_node");

    // Get node handle
    ros::NodeHandle nodeHandle;

    // Create subscriber
    ros::Subscriber odomSubscriber = nodeHandle.subscribe("/odom", 20, odomCallback);

    // Create publisher
    ros::Publisher eulerMessagePublisher = nodeHandle.advertise<md25::StampedEuler>("/euler", 20);
    eulerMessagePublisherPtr = & eulerMessagePublisher;

    // Spin
    ros::spin();

    return 0;

}

