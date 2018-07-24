/*
 yolo_detector.cpp

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

#include "yolo_detector.h"
#include "cv_conversion.h"

#include <cv_bridge/cv_bridge.h>

YoloDetector::YoloDetector(ros::NodeHandle & _nodeHandle, ros::NodeHandle & _privateNodeHandle) :
    nodeHandle(_nodeHandle), privateNodeHandle(_privateNodeHandle), imageTransport(nodeHandle) {
}

int YoloDetector::init() {

    std::string darknetHomeFolder;

    char * rosHomeEnv;

    if ((rosHomeEnv = getenv("ROS_HOME"))) {

        // Use ROS_HOME environment variable
        darknetHomeFolder = rosHomeEnv;

    } else if ((rosHomeEnv = getenv("HOME"))) {

        // Use .ros folder HOME environment variable
        darknetHomeFolder = rosHomeEnv;
        darknetHomeFolder += "/.ros";

    }

    // Add darknet folder
    darknetHomeFolder += "/darknet/";

    // Log
    ROS_INFO("darknet home detected in: %s", darknetHomeFolder.c_str());

    // Get node parameters
    std::string configFile;
    std::string weightsFile;
    std::string namesFile;
    privateNodeHandle.param<std::string>("config_file", configFile, "yolov3-tiny.cfg");
    privateNodeHandle.param<std::string>("weights_file", weightsFile, "yolov3-tiny.weights");
    privateNodeHandle.param<std::string>("names_file", namesFile, "coco.names");
    privateNodeHandle.param<float>("threshold", threshold, 0.5f);
    privateNodeHandle.param<float>("hier_threshold", hierThreshold, 0.5f);
    privateNodeHandle.param<float>("nms_threshold", nmsThreshold, 0.45f);

    // Log
    ROS_INFO("config file: %s", configFile.c_str());
    ROS_INFO("weights file: %s", weightsFile.c_str());
    ROS_INFO("names file: %s", namesFile.c_str());
    ROS_INFO("threshold: %g", threshold);
    ROS_INFO("hier threshold: %g", hierThreshold);
    ROS_INFO("nms threshold: %g", nmsThreshold);

    // Log
    ROS_INFO("loading network...");

    // Load darknet network
    std::string configFileURL = darknetHomeFolder + configFile;
    char * configFileChar = new char[configFileURL.length() + 1];
    strcpy(configFileChar, configFileURL.c_str());
    std::string weightsFileURL = darknetHomeFolder + weightsFile;
    char * weightsFileChar = new char[weightsFileURL.length() + 1];
    strcpy(weightsFileChar, weightsFileURL.c_str());
    darknetNetwork = load_network(configFileChar, weightsFileChar, 0);
    set_batch_network(darknetNetwork, 1);

    // Initialize random number generator
    srand(2222222);

    // Log
    ROS_INFO("loading labels...");

    // Load labels
    std::string namesFileURL = darknetHomeFolder + namesFile;
    char * namesFileChar = new char[namesFileURL.length() + 1];
    strcpy(namesFileChar, namesFileURL.c_str());
    labels = get_labels(namesFileChar);

    // Log
    ROS_INFO("network setup completed");

    // Create image subscriber (set compressed transport level as default)
    imageSubscriber = imageTransport.subscribe("image", 1, & YoloDetector::imageCallback, this);

    // Create detection array publisher
    detectionArrayPublisher = nodeHandle.advertise<darknet_cv::DetectionArray>("detections", 5);

    // Create debug image publisher
    debugImagePublisher = imageTransport.advertise("image_debug", 1);

    return 0;

}

void YoloDetector::imageCallback(const sensor_msgs::ImageConstPtr & imageConstPtr) {

    cv_bridge::CvImagePtr cvImagePtr;

    try {

        // Convert image message to OpenCV image
        cvImagePtr = cv_bridge::toCvCopy(imageConstPtr, "");

        // Convert image to darknet image
        image darknetImage = toImage(cvImagePtr);

        // Resize image according to network size
        image resizedImage = letterbox_image(darknetImage, darknetNetwork->w, darknetNetwork->h);

        // Network predict
        float * data = resizedImage.data;
        network_predict(darknetNetwork, data);

        // Get network detections
        int boxesCount = 0;
        detection * detections = get_network_boxes(darknetNetwork, darknetImage.w, darknetImage.h, threshold, hierThreshold, 0, 1, & boxesCount);

        if (nmsThreshold > 0) {

            // Get number of classes
            int classes = darknetNetwork->layers[darknetNetwork->n - 1].classes;

            // Check for redundant
            do_nms_sort(detections, boxesCount, classes, nmsThreshold);

        }

        // Prepare detection results
        darknet_cv::DetectionArray detectionArrayMsg;

        // Use same header from image message
        detectionArrayMsg.header = imageConstPtr->header;

        // Calculate detection numbers
        int detectionCount = 0;
        for (int i = 0; i < boxesCount; i++) {
            for (int j = 0; j < detections[i].classes; j++) {
                if (detections[i].prob[j] > 0) {
                    detectionCount++;
                }
            }
        }

        // Resize for detection count
        detectionArrayMsg.detections.resize(detectionCount);

        // Create message
        int detectionIndex = 0;
        for (int i = 0; i < boxesCount; i++) {
            for (int j = 0; j < detections[i].classes; j++) {
                if (detections[i].prob[j] > 0) {

                    // Create message
                    detectionArrayMsg.detections[detectionIndex].detectionClass.id = j;
                    detectionArrayMsg.detections[detectionIndex].detectionClass.label = labels[j];
                    detectionArrayMsg.detections[detectionIndex].probability = detections[i].prob[j];
                    detectionArrayMsg.detections[detectionIndex].roi.x_offset = (detections[i].bbox.x - detections[i].bbox.w / 2) * imageConstPtr->width;
                    detectionArrayMsg.detections[detectionIndex].roi.y_offset = (detections[i].bbox.y - detections[i].bbox.h / 2) * imageConstPtr->height;
                    detectionArrayMsg.detections[detectionIndex].roi.height = detections[i].bbox.h * imageConstPtr->height;
                    detectionArrayMsg.detections[detectionIndex].roi.width = detections[i].bbox.w * imageConstPtr->width;

                    // Increase index
                    detectionIndex++;

                }
            }
        }

        // Public message
        detectionArrayPublisher.publish(detectionArrayMsg);

        if (debugImagePublisher.getNumSubscribers() > 0) {

            // Publish debug image
            publishDebugImage(imageConstPtr, detectionArrayMsg);

        }


    } catch (cv_bridge::Exception & ex) {

        // Log
        ROS_ERROR("an error occurs converting image to OpenCV image: %s", ex.what());

        return;

    }

}

void YoloDetector::publishDebugImage(const sensor_msgs::ImageConstPtr & imageConstPtr, const darknet_cv::DetectionArray & detectionArrayMsg) {

    cv_bridge::CvImagePtr cvImagePtr;

    try {

        // Convert image message to OpenCV image
        cvImagePtr = cv_bridge::toCvCopy(imageConstPtr, "");

    } catch (cv_bridge::Exception & ex) {

        // Log
        ROS_ERROR("an error occurs converting image to OpenCV image: %s", ex.what());

        return;

    }

    // Get detection count
    int detectionCount = detectionArrayMsg.detections.size();

    for (int j = 0; j < detectionCount; j++) {

        // Get detection region of interest
        sensor_msgs::RegionOfInterest roi = detectionArrayMsg.detections[j].roi;

        // Get label
        const char * label = detectionArrayMsg.detections[j].detectionClass.label.c_str();

        // Draw rectangle
        cv::rectangle(cvImagePtr->image, toCvRect(roi), cv::Scalar(110, 220, 0),  2, 8);

        // Draw text
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, 0);
        cv::putText(cvImagePtr->image, label, toCvTopTextBasePoint(roi, textSize), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 110, 220, 0 ));

    }

    // Convert to image message
    sensor_msgs::Image debugImageMsg;
    cvImagePtr->toImageMsg(debugImageMsg);

    // Publish image message
    debugImagePublisher.publish(debugImageMsg);


}
