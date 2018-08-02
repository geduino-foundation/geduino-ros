/*
 qr_detector.h

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

#ifndef QR_DETECTOR_H_
#define QR_DETECTOR_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <zxing_cv/QRCodeArray.h>
#include <opencv2/core/core.hpp>
#include <zxing/Result.h>
#include <zxing/Reader.h>
#include <zxing_cv/QRDetectorConfig.h>

class QRDetector {

	public:

        QRDetector(ros::NodeHandle & _nodeHandle, ros::NodeHandle & _privateNodeHandle);

        int init();

    private:

        void reconfigureCallback(zxing_cv::QRDetectorConfig & config, uint32_t level);

        void imageCallback(const sensor_msgs::ImageConstPtr & imageConstPtr, const sensor_msgs::CameraInfoConstPtr & cameraInfoPtr);

        void publishDebugImage(const sensor_msgs::ImageConstPtr & imageConstPtr, const zxing::Ref<zxing::Result> & result);

        ros::NodeHandle nodeHandle;
        ros::NodeHandle privateNodeHandle;

        dynamic_reconfigure::Server<zxing_cv::QRDetectorConfig> * dynamicReconfigureServer;

        zxing::Ref<zxing::Reader> qrReader;

        int adaptiveThresholdBlockSize;
        int adaptiveThresholdThreshold;

        image_transport::ImageTransport imageTransport;
        image_transport::CameraSubscriber imageSubscriber;
        ros::Publisher qrCodeArrayPublisher;
        image_transport::Publisher optimizedImagePublisher, debugImagePublisher;

};

#endif
