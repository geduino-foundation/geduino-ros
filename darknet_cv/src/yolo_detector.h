/*
 yolo_detector.h

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

#ifndef YOLO_DETECTOR
#define YOLO_DETECTOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <darknet_cv/DetectionArray.h>

extern "C" {
    #include "darknet/include/darknet.h"
}

class YoloDetector {

public:

    YoloDetector(ros::NodeHandle & _nodeHandle, ros::NodeHandle & _privateNodeHandle);

    int init();

private:

    void imageCallback(const sensor_msgs::ImageConstPtr & imageConstPtr);

    void publishDebugImage(const sensor_msgs::ImageConstPtr & imageConstPtr, const darknet_cv::DetectionArray & detectionArrayMsg);

    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;
    ros::Publisher detectionArrayPublisher;
    image_transport::Publisher debugImagePublisher;

    network * darknetNetwork;
    char ** labels;

    float threshold;
    float hierThreshold;
    float nmsThreshold;

};

#endif

