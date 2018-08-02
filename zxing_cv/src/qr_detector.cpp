/*
 qr_detector.cpp

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

#include "qr_detector.h"
#include "cv_conversion.h"

#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zxing/Binarizer.h>
#include <zxing/ReaderException.h>
#include <zxing/Exception.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>
#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/MatSource.h>

QRDetector::QRDetector(ros::NodeHandle & _nodeHandle, ros::NodeHandle & _privateNodeHandle) :
    nodeHandle(_nodeHandle), privateNodeHandle(_privateNodeHandle), imageTransport(nodeHandle) {
}

int QRDetector::init() {

    // Create dynamic reconfigure server
    dynamicReconfigureServer = new dynamic_reconfigure::Server<zxing_cv::QRDetectorConfig>(privateNodeHandle);
    dynamic_reconfigure::Server<zxing_cv::QRDetectorConfig>::CallbackType callback = boost::bind(& QRDetector::reconfigureCallback, this, _1, _2);
    dynamicReconfigureServer->setCallback(callback);

    // Create image subscriber
    imageSubscriber = imageTransport.subscribeCamera("/camera/image", 1, & QRDetector::imageCallback, this);

    // Create optimized image publisher
    optimizedImagePublisher = imageTransport.advertise("image_optimized", 1);

    // Create debug image publisher
    debugImagePublisher = imageTransport.advertise("image_debug", 1);

    // Create qr codes publisher
    qrCodeArrayPublisher = nodeHandle.advertise<zxing_cv::QRCodeArray>("qr_codes", 5);

    // Set barcode reader
    qrReader.reset(new zxing::qrcode::QRCodeReader);

    return 0;

}

void QRDetector::reconfigureCallback(zxing_cv::QRDetectorConfig & config, uint32_t level) {

    // Update configuration
    adaptiveThresholdBlockSize = config.adaptive_threshold_block_size;
    adaptiveThresholdThreshold = config.adaptive_threshold_threshold;

    if (adaptiveThresholdBlockSize % 2 == 0) {

        // Increae by one (adaptive threshold block size must be an odd number
        adaptiveThresholdBlockSize++;

    }

}

void QRDetector::imageCallback(const sensor_msgs::ImageConstPtr & imageConstPtr, const sensor_msgs::CameraInfoConstPtr & cameraInfoPtr) {

    cv_bridge::CvImagePtr cvImagePtr;

    try {

        // Convert image message to OpenCV image
        cvImagePtr = cv_bridge::toCvCopy(imageConstPtr, "");

    } catch (cv_bridge::Exception & ex) {

        // Log
        ROS_ERROR("an error occurs converting image to OpenCV image: %s", ex.what());

        return;

    }

    // Convert to grayscale
    cv::cvtColor(cvImagePtr->image, cvImagePtr->image, CV_BGR2GRAY);
    cvImagePtr->encoding = sensor_msgs::image_encodings::MONO8; // Image encodin on CvImage must changed too!

    // Apply adaptive threshold
    cv::adaptiveThreshold(cvImagePtr->image, cvImagePtr->image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, adaptiveThresholdBlockSize, adaptiveThresholdThreshold);

    if (optimizedImagePublisher.getNumSubscribers() > 0) {

        // Publish optimized image message
        optimizedImagePublisher.publish(cvImagePtr->toImageMsg());

    }

    // Create luminance source
    zxing::Ref<zxing::LuminanceSource> source = zxing::MatSource::create(cvImagePtr->image);

    try {

        // Read qr code
        zxing::Ref<zxing::Binarizer> binarizer(new zxing::GlobalHistogramBinarizer(source));
        zxing::Ref<zxing::BinaryBitmap> bitmap(new zxing::BinaryBitmap(binarizer));
        zxing::Ref<zxing::Result> result(qrReader->decode(bitmap, zxing::DecodeHints(zxing::DecodeHints::TRYHARDER_HINT)));

        // Create qr codes message
        zxing_cv::QRCodeArray qrCodeArrayMessage;
        qrCodeArrayMessage.header = cvImagePtr->header;
        qrCodeArrayMessage.qr_codes.resize(1);
        qrCodeArrayMessage.qr_codes[0].content = result->getText()->getText();

        // Publish qr code message
        qrCodeArrayPublisher.publish(qrCodeArrayMessage);

        if (debugImagePublisher.getNumSubscribers() > 0) {

            // Publish debug image
            publishDebugImage(imageConstPtr, result);

        }

    } catch (const zxing::ReaderException & e) {

        if (debugImagePublisher.getNumSubscribers() > 0) {

            zxing::Ref<zxing::Result> result;

            // Publish debug image
            publishDebugImage(imageConstPtr, result);

        }

    } catch (const zxing::IllegalArgumentException & e) {

        // Log
        ROS_ERROR("zxing illegal argument exception: %s", e.what());

    } catch (const zxing::Exception & e) {

        // Log
        ROS_ERROR("zxing reader exception: %s", e.what());

    } catch (const cv::Exception & e) {

        // Log
        ROS_ERROR("zxing reader exception: %s", e.what());

    }

}

void QRDetector::publishDebugImage(const sensor_msgs::ImageConstPtr & imageConstPtr, const zxing::Ref<zxing::Result> & result) {

    cv_bridge::CvImagePtr cvImagePtr;

    try {

        // Convert image message to OpenCV image
        cvImagePtr = cv_bridge::toCvCopy(imageConstPtr, "");

    } catch (cv_bridge::Exception & ex) {

        // Log
        ROS_ERROR("an error occurs converting image to OpenCV image: %s", ex.what());

        return;

    }

    // Get result point count
    int resultPointCount = (result == NULL) ? 0 : result->getResultPoints()->size();

    for (int j = 0; j < resultPointCount; j++) {

        // Draw circle
        cv::circle(cvImagePtr->image, toCvPoint(result->getResultPoints()[j]), 10, cv::Scalar( 110, 220, 0 ), 2);

    }

    // Draw boundary on image
    if (resultPointCount > 1) {

        for (int j = 0; j < resultPointCount; j++) {

            // Get start result point
            zxing::Ref<zxing::ResultPoint> previousResultPoint = (j > 0) ? result->getResultPoints()[j - 1] : result->getResultPoints()[resultPointCount - 1];

            // Draw line
            cv::line(cvImagePtr->image, toCvPoint(previousResultPoint), toCvPoint(result->getResultPoints()[j]), cv::Scalar( 110, 220, 0 ),  2, 8 );

            // Create text
            char indexStr[7];
            sprintf(indexStr, "Point %d", j);

            // Draw text
            cv::putText(cvImagePtr->image, indexStr, toCvPoint(result->getResultPoints()[j]), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 110, 220, 0 ));

            // Update previous point
            previousResultPoint = result->getResultPoints()[j];

        }

    }

    if (result != NULL) {

        // Draw text
        cv::putText(cvImagePtr->image, result->getText()->getText(), toCvPoint(result->getResultPoints()[0]), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 110, 220, 0 ));

    }

    // Convert to image message
    sensor_msgs::Image debugImageMsg;
    cvImagePtr->toImageMsg(debugImageMsg);

    // Publish image message
    debugImagePublisher.publish(debugImageMsg);

}
