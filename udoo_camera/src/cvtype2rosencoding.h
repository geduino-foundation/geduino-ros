/*
 cvtype2rosencoding.c√®pp

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

// The OpenCV types
int32_t OPENCV_TYPE_ENUM[] = {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
	CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
	CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
	CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
	CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
	CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
	CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

// The ROS encodings
std::string ROS_ENCODING_ENUM[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
	"CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
	"CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
	"CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
	"CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
	"CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
	"CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

#define OPENCV_TYPE_LENGHT 35

/**
 Convert image type returned by cv::Mat::type() to ROS image encoding according to
 sensor_msgs::image_encodings. Empty string is returned if unknown image type is given.
√*/
std::string toROSEncoding(int32_t cvType) {


	// Iterate over OpenCV type enums
	for(int32_t index = 0; index < OPENCV_TYPE_LENGHT; index++) {

		// Check cv type
		if (cvType == OPENCV_TYPE_ENUM[index]) {

			// Matched!
			return ROS_ENCODING_ENUM[index];

		}

	}

	return "";

}
