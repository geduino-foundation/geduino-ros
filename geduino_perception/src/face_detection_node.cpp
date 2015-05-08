/*
 face_detection_node.cpp

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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geduino_perception_msgs/StampedFaces.h>

// The node parameters
std::string haarCascadeFile;
double scaleFactor;
int minNeighbors;
int minWidth;
int minHeight;
int maxWidth;
int maxHeight;

// The subscribed image topic
std::string imageTopic;

// The faces publisher
ros::Publisher * facesPublisherPtr;

// The cascade classifiers
cv::CascadeClassifier * cascadeClassifierPtr;

void imageCallback(const sensor_msgs::ImageConstPtr & imageConstPtr) {

	cv_bridge::CvImagePtr cvImagePtr;

	try {

		// Convert image message to OpenCV image
		cvImagePtr = cv_bridge::toCvCopy(imageConstPtr, "");

	} catch (cv_bridge::Exception & ex) {

		// Log
		ROS_ERROR("an error occurs converting image to OpenCV image: %s", ex.what());

		return;

	}

	// Convart to grayscale and equalize histogram
	cv::Mat grayscaleImage;
	cv::cvtColor(cvImagePtr->image, grayscaleImage, CV_BGR2GRAY);
	cv::equalizeHist(grayscaleImage, grayscaleImage);

	// Detect frontal faces
	std::vector<cv::Rect> faces;
	cascadeClassifierPtr->detectMultiScale(grayscaleImage, faces, scaleFactor, minNeighbors, 0, cv::Size(minWidth, minHeight));

	// Create faces message
	geduino_perception_msgs::StampedFaces facesMessage;
	facesMessage.header.stamp = imageConstPtr->header.stamp;
	facesMessage.faces.image_topic = imageTopic;
	facesMessage.faces.rois.resize(faces.size());

	for (size_t index = 0; index < faces.size(); index++) {

		// Get face
		cv::Rect face = faces[index];

		// Set message
		facesMessage.faces.rois[index].x_offset = face.x;
		facesMessage.faces.rois[index].y_offset = face.y;
		facesMessage.faces.rois[index].width = face.width;
		facesMessage.faces.rois[index].height = face.height;

	}

	// Publish detected images
	facesPublisherPtr->publish(facesMessage);

}

int main(int argc, char** argv) {

	// Init ROS
	ros::init(argc, argv, "face_detection_node");

	// Get node handle
	ros::NodeHandle nodeHandle;

	// Get private node handle
	ros::NodeHandle privateNodeHandle("~");

	// Get node parameters
	privateNodeHandle.param<std::string>("haar_cascade_file", haarCascadeFile, "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml");
	privateNodeHandle.param("scale_factor", scaleFactor, 1.1);
	privateNodeHandle.param("min_neighbors", minNeighbors, 3);
	privateNodeHandle.param("min_width", minWidth, 50);
	privateNodeHandle.param("min_height", minHeight, 50);
	privateNodeHandle.param("max_width", maxWidth, 0);
	privateNodeHandle.param("max_height", maxHeight, 0);

	// Log
	ROS_INFO("haar cascade file: %s", haarCascadeFile.c_str());
	ROS_INFO("scale factor: %g", scaleFactor);
	ROS_INFO("min neighbors: %d", minNeighbors);
	ROS_INFO("min width: %d", minWidth);
	ROS_INFO("min height: %d", minHeight);
	ROS_INFO("max width: %d", maxWidth);
	ROS_INFO("max height: %d", maxHeight);

	// Create image transport
	image_transport::ImageTransport imageTransport(nodeHandle);

	// Create image subscriber
	image_transport::Subscriber imageSubscriber = imageTransport.subscribe("camera/image", 1, imageCallback);
	imageTopic = imageSubscriber.getTopic();

	// Create faces publisher
	ros::Publisher facesPublisher = nodeHandle.advertise<geduino_perception_msgs::StampedFaces>("perception/faces", 1);
	facesPublisherPtr = & facesPublisher;

	// Create ascade classifier
	cv::CascadeClassifier cascadeClassifier(haarCascadeFile);
	cascadeClassifierPtr = & cascadeClassifier;

	if (cascadeClassifier.empty()) {

		// Log
		ROS_FATAL("load cascade file failed: %s", haarCascadeFile.c_str());

		return -1;

	}

	// Spin
	ros::spin();

}
