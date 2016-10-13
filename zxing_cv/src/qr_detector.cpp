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
#include <opencv2/calib3d/calib3d.hpp>
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

    // Init rvec and tvec
    rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    tvec.at<double>(2) = 1;

    // Get node parameters
    privateNodeHandle.param("adaptive_threshold_block_size", adaptiveThresholdBlockSize, 201);
    privateNodeHandle.param("adaptive_threshold_threshold", adaptiveThresholdThreshold, 20);
    std::vector<double> qrCodePointsParam;
    privateNodeHandle.getParam("qr_code_points", qrCodePointsParam);
    privateNodeHandle.param("ignore_point_in_excess", ignorePointsInExcess, true);
    privateNodeHandle.getParam("marker_scale", markerScale);
    privateNodeHandle.getParam("marker_point_color", markerPointColor);
    privateNodeHandle.getParam("marker_text_color", markerTextColor);

    if (qrCodePointsParam.size() % 3 != 0) {

        // Log error
        ROS_FATAL("qr_code_points size must be multiple of 3 (x, y, z), actual: %lu", qrCodePointsParam.size());

        return -1;

    } if (qrCodePointsParam.size() < 9) {

        // Log error
        ROS_FATAL("qr_code_points size must contain at least 3 points, i.e. 9 values (x, y, z), actual: %lu", qrCodePointsParam.size());

        return -1;

    }

    // Convert double vector to point3f vector
    qrCodePoints = toCvPoint3fVector(qrCodePointsParam);

    if (markerScale.size() == 0) {

        // Use default
        markerScale.push_back(0.1);
        markerScale.push_back(0.1);
        markerScale.push_back(0.1);

    } else if (markerScale.size() != 3) {

        // Log error
        ROS_FATAL("marker_scale size must be 3 (x, y, z), actual: %lu", markerScale.size());

        return -1;

    }

    if (markerPointColor.size() == 0) {

        // Use default
        markerPointColor.push_back(1);
        markerPointColor.push_back(0);
        markerPointColor.push_back(0);
        markerPointColor.push_back(0.5);

    } else if (markerPointColor.size() != 4) {

        // Log error
        ROS_FATAL("marker_point_color size must be 4 (r, g, b, a), actual: %lu", markerPointColor.size());

        return -1;

    }

    if (markerTextColor.size() == 0) {

        // Use default
        markerTextColor.push_back(0);
        markerTextColor.push_back(1);
        markerTextColor.push_back(1);
        markerTextColor.push_back(1);

    } else if (markerTextColor.size() != 4) {

        // Log error
        ROS_FATAL("tmarker_ext_color size must be 4 (r, g, b, a), actual: %lu", markerTextColor.size());

        return -1;

    }

    // Create image subscriber
    imageSubscriber = imageTransport.subscribeCamera("/camera/image", 1, & QRDetector::imageCallback, this);

    // Create optimized image publisher
    optimizedImagePublisher = imageTransport.advertise("image_optimized", 1);

    // Create debug image publisher
    debugImagePublisher = imageTransport.advertise("image_debug", 1);

    // Create qr codes publisher
    qrCodeArrayPublisher = nodeHandle.advertise<zxing_cv::QRCodeArray>("qr_codes", 5);

    // Create qr codes marker publisher
    markerArrayPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("markers", 5);

    // Set barcode reader
    qrReader.reset(new zxing::qrcode::QRCodeReader);

    return 0;

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

        if (!ignorePointsInExcess && result->getResultPoints()->size() != qrCodePoints.size()) {

            // Log
            ROS_ERROR("detected %d points on qrcode, but expected was %lu", result->getResultPoints()->size(), qrCodePoints.size());

            return;

        }

        // The detected projection of qr code points
        std::vector<cv::Point2f> imagePoints = toCvPoint2fVector(result->getResultPoints(), qrCodePoints.size());

        // The camera matrix
        cv::Mat cameraMatrix = toCvMat3x3(cameraInfoPtr->K);

        // The distorsion coefficients
        cv::Mat distCoeffs;

        if (cameraInfoPtr->distortion_model == "plumb_bob") {

            // Get distorsion coefficients from camera info
            distCoeffs = toCvMat5x1(cameraInfoPtr->D);

        } else {

            // Set distorsion coefficients to zeroes (i.e. no distorsion)
            distCoeffs = cv::Mat::zeros(5, 1, cv::DataType<double>::type);

            // Log
            ROS_WARN("unknow distorsion model %s", cameraInfoPtr->distortion_model.c_str());

        }

        // Solve projection
        if (cv::solvePnP(qrCodePoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, true, 0)) {

            // Create qr codes message
            zxing_cv::QRCodeArray qrCodeArrayMessage;
            qrCodeArrayMessage.header = cvImagePtr->header;
            qrCodeArrayMessage.qr_codes.resize(1);
            qrCodeArrayMessage.qr_codes[0].pose.position.x = tvec.at<double>(0);
            qrCodeArrayMessage.qr_codes[0].pose.position.y = tvec.at<double>(1);
            qrCodeArrayMessage.qr_codes[0].pose.position.z = tvec.at<double>(2);
            qrCodeArrayMessage.qr_codes[0].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
            qrCodeArrayMessage.qr_codes[0].content = result->getText()->getText();

            // Publish qr code message
            qrCodeArrayPublisher.publish(qrCodeArrayMessage);

            // Publish markers
            publishMarkers(qrCodeArrayMessage);

        } else {

            // Log
            ROS_WARN("solve PNP failed");

        }

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

void QRDetector::publishMarkers(const zxing_cv::QRCodeArray & qrCodeArrayMsg) {

    // Create marker array message
    visualization_msgs::MarkerArray markerArrayMsg;
    markerArrayMsg.markers.resize(qrCodeArrayMsg.qr_codes.size() * 2);

    for (int index = 0; index < qrCodeArrayMsg.qr_codes.size(); index++) {

        // Add point marker
        int markerIndex = index * 2;
        markerArrayMsg.markers[markerIndex].header = qrCodeArrayMsg.header;
        markerArrayMsg.markers[markerIndex].ns = "zxing";
        markerArrayMsg.markers[markerIndex].id = markerIndex;
        markerArrayMsg.markers[markerIndex].type = visualization_msgs::Marker::POINTS;
        markerArrayMsg.markers[markerIndex].action = visualization_msgs::Marker::ADD;
        markerArrayMsg.markers[markerIndex].scale.x = markerScale.at(0);
        markerArrayMsg.markers[markerIndex].scale.y = markerScale.at(1);
        markerArrayMsg.markers[markerIndex].scale.z = markerScale.at(2);
        markerArrayMsg.markers[markerIndex].points.resize(1);
        markerArrayMsg.markers[markerIndex].colors.resize(1);
        markerArrayMsg.markers[markerIndex].points[0].x = qrCodeArrayMsg.qr_codes[index].pose.position.x;
        markerArrayMsg.markers[markerIndex].points[0].y = qrCodeArrayMsg.qr_codes[index].pose.position.y;
        markerArrayMsg.markers[markerIndex].points[0].z = qrCodeArrayMsg.qr_codes[index].pose.position.z;
        markerArrayMsg.markers[markerIndex].colors[0].r = markerPointColor.at(0);
        markerArrayMsg.markers[markerIndex].colors[0].g = markerPointColor.at(1);
        markerArrayMsg.markers[markerIndex].colors[0].b = markerPointColor.at(2);
        markerArrayMsg.markers[markerIndex].colors[0].a = markerPointColor.at(3);

        // Add text marker
        markerIndex++;
        markerArrayMsg.markers[markerIndex].header = qrCodeArrayMsg.header;
        markerArrayMsg.markers[markerIndex].ns = "zxing";
        markerArrayMsg.markers[markerIndex].id = markerIndex;
        markerArrayMsg.markers[markerIndex].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        markerArrayMsg.markers[markerIndex].action = visualization_msgs::Marker::ADD;
        markerArrayMsg.markers[markerIndex].scale.x = markerScale.at(0);
        markerArrayMsg.markers[markerIndex].scale.y = markerScale.at(1);
        markerArrayMsg.markers[markerIndex].scale.z = markerScale.at(2);
        markerArrayMsg.markers[markerIndex].pose= qrCodeArrayMsg.qr_codes[index].pose;
        markerArrayMsg.markers[markerIndex].color.r = markerTextColor.at(0);
        markerArrayMsg.markers[markerIndex].color.g = markerTextColor.at(1);
        markerArrayMsg.markers[markerIndex].color.b = markerTextColor.at(2);
        markerArrayMsg.markers[markerIndex].color.a = markerTextColor.at(3);
        markerArrayMsg.markers[markerIndex].text = qrCodeArrayMsg.qr_codes[index].content;

    }

    // Publish markers message
    markerArrayPublisher.publish(markerArrayMsg);
}
