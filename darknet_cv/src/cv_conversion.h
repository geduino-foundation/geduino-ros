/*
 cv_conversion.h

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

#ifndef CV_CONVERSION
#define CV_CONVERSION

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/RegionOfInterest.h>

image toImage(cv_bridge::CvImagePtr cvImagePtr){

    // Clone CV image from image pointer
    cv::Mat cvImageCopy = cvImagePtr->image.clone();

    // Create IPL image
    IplImage * iplImage = new IplImage(cvImageCopy);

    // Get image paramters
    int height = iplImage->height;
    int width = iplImage->width;
    int channels = iplImage->nChannels;
    int step = iplImage->widthStep;

    // Get pointer to image data
    unsigned char * imageData = (unsigned char *) iplImage->imageData;

    // Create image
    image darknetImage = make_image(width, height, channels);

    // Convertion
    for (int i = 0; i < height; ++i) {
        for (int k= 0; k < channels; ++k) {
            for (int j = 0; j < width; ++j) {
                darknetImage.data[k * width * height + i * width + j] = imageData[i * step + j * channels + k]/255.0;
            }
        }
    }


    return darknetImage;

}

cv::Rect toCvRect(const sensor_msgs::RegionOfInterest & roi) {
    return cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height);
}

cv::Point toCvTopTextBasePoint(const sensor_msgs::RegionOfInterest & roi, cv::Size size) {
    return cv::Point(roi.x_offset, roi.y_offset - size.height);
}

#endif

