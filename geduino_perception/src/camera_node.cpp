/*
 camera_node.cpp

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
 camera_node is a ROS node implementation that publish frames captured at
 given frequency from given cam and publish it as sensor_msgs/Image message.

 Publish:
 - camera/image (sensor_msgs/Image)√: the image from camera.

 √√Parameters
 - device_id, the id of the captured video device (default: 0, means first available);
 - camera_frame: the frame attached to captured camera (default: camera_link). See
   sensor_msgs/Image for orientation guideline;
 - fps√, the frames per seconds.
*/

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

#include "cvtype2rosencoding.h"

// The node parameters
int deviceId;
std::string cameraFrame;
double fps;

int main(int argc, char** argv) {

	// Init ROS
	ros::init(argc, argv, "camera_node");

	// Get node handle
	ros::NodeHandle nodeHandle;

	// Get private node handle
	ros::NodeHandle privateNodeHandle("~");

	// Get node parameters
	privateNodeHandle.param("device_id", deviceId, 0);
	privateNodeHandle.param<std::string>("camera_frame", cameraFrame, "camera_link");
	privateNodeHandle.param("fps", fps, 10.0);

	// Log
	ROS_INFO("device id: %d", deviceId);
	ROS_INFO("camera frame: %s", cameraFrame.c_str());
	ROS_INFO("frames per second: %g", fps);

	// Create image transport
	image_transport::ImageTransport imageTransport(nodeHandle);

	// Create image publisher
	image_transport::Publisher imagePublisher = imageTransport.advertise("camera/image", 1);

	// Open video captire
	cv::VideoCapture videoCapture(deviceId);

	if (!videoCapture.isOpened()) {

		// Log
		ROS_FATAL("open video capture failed on device id: %d", deviceId);

		return -1;

	}

	// The captured image
	cv::Mat image;

	// Creae rate
	ros::Rate rate(fps);

	while (ros::ok()) {

		// Sleep for rate
		rate.sleep();

		// Capture image
		bool result = videoCapture.read(image);

		if (result) {

			// Get image type
			int32_t type = image.type();

			// Convert to image encoding
			std::string encoding = toROSEncoding(type);

			if (encoding.length() == 0) {

				// Log
				ROS_WARN("unknow image encoding for OpenCV type: %d", type);

			}

			// Create message header
			std_msgs::Header header;
			header.frame_id = cameraFrame;

			// Create cv image
			cv_bridge::CvImage cvImage(header, encoding, image);

			// Convert to image message
			sensor_msgs::Image imageMsg;
			cvImage.toImageMsg(imageMsg);

			// Publish image message
			imagePublisher.publish(imageMsg);

		} else {

			// Log
			ROS_WARN("read image from capture failed");

		}

		// Spin
		ros::spinOnce();


	}

	// Release video capture
	videoCapture.release();

}
