/*
 qr_detector_nodelet.cpp

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

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "qr_detector.h"

namespace zxing_cv {

class QRDetectorNodelet : public nodelet::Nodelet {

public:

    QRDetectorNodelet() : running(false) {};

    ~QRDetectorNodelet() {

        if (running) {

            // Set running to false
            running = false;

            // Join thread
            thread->join();

        }

    }

private:

    virtual void onInit() {

        // Get node handle
        ros::NodeHandle nodeHandle = getPrivateNodeHandle();

        // Create qr detector
        qrDetector.reset(new QRDetector(nodeHandle, nodeHandle));

        // Init qr detector
        qrDetector->init();

        // Set running to true
        running = true;

        // Create nodelet thread
        thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(& QRDetectorNodelet::main, this)));

    }

    void main() {

        while (running) {

            // Process
            //qrDetector->process();

        }

    }

    bool running;

    boost::shared_ptr<QRDetector> qrDetector;

    boost::shared_ptr<boost::thread> thread;

};

}

PLUGINLIB_DECLARE_CLASS(zxing_cv, QRDetectorNodelet, zxing_cv::QRDetectorNodelet, nodelet::Nodelet);
