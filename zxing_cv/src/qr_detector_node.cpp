/*
 qr_detector_node.cpp

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

#include <ros/ros.h>

#include "qr_detector.h"

int main(int argc, char** argv) {

    // Init ROS
    ros::init(argc, argv, "qr_detector_node");

    // Get node handle
    ros::NodeHandle nodeHandle;

    // Get private node handle
    ros::NodeHandle privateNodeHandle("~");

    // Create qr detector
    QRDetector qrDetector(nodeHandle, privateNodeHandle);

    // Init qr detector
    int result = qrDetector.init();

    if (result != 0) {

        // Log
        ROS_FATAL("init failed with return code %d", result);

        return result;

    }

    // Spin
    ros::spin();

}
