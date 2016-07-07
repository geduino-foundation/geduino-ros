/*
 range_layer.cpp
    
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

#include <range_layer/range_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(range_layer::RangeLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace range_layer {

RangeLayer::RangeLayer() {}

void RangeLayer::onInitialize() {

    // Create node handle
    ros::NodeHandle nodeHandle("~/" + name_);

    // Set current to true
    current_ = true;

    // Create dynamic reconfigure server
    dynamicReconfigureServer = new dynamic_reconfigure::Server<RangeLayerConfig>(nodeHandle);
    dynamic_reconfigure::Server<RangeLayerConfig>::CallbackType callback = boost::bind(& RangeLayer::reconfigureCallback, this, _1, _2);
    dynamicReconfigureServer->setCallback(callback);

    // Get params
    std::vector<std::string> topics;
    nodeHandle.getParam("topics", topics);

    if (topics.size() < 1) {

        // Log
        ROS_WARN("No topics specified: this layer will have not effect in costmap");

    }

    for (int index = 0; index < topics.size(); index++) {

        // Log
        ROS_INFO("Subscribing topic %s", topics[index].c_str());

        // Subscribe range topics
        boost::function<void (const sensor_msgs::Range & rangeConstPtr)> rangeCallbackFunction;
        rangeCallbackFunction = boost::bind(& RangeLayer::rangeCallback, this, _1, topics[index]);

        // Create subscriber
        ros::Subscriber subscriber = nodeHandle.subscribe<sensor_msgs::Range>(topics[index], 1, rangeCallbackFunction);

        // Add to range subscribers
        rangeSubscribers.push_back(subscriber);

    }

}


void RangeLayer::reconfigureCallback(RangeLayerConfig & config, uint32_t level) {

    // Update configuration
    enabled_ = config.enabled;
    readingsTimeout = config.readings_timeout;
    tfTimeout = config.tf_timeout;

}

void RangeLayer::rangeCallback(const sensor_msgs::Range & rangeMsg, const std::string & rangeTopic) {

    // Lock ranges mutex before add new range message
    boost::mutex::scoped_lock lock(rangesMutex);

    // Put range in ranges map
    ranges[rangeTopic] = rangeMsg;

}

void RangeLayer::updatePoints() {

    std::vector<sensor_msgs::Range> rangesCopy;

    // Lock ranges mutex before copying
    rangesMutex.lock();

    for (std::map<std::string, sensor_msgs::Range>::iterator iterator = ranges.begin(); iterator != ranges.end(); ++iterator) {

        // Put range message in copy vector
        rangesCopy.push_back(iterator->second);

    }

    // Unlock ranges mutex
    rangesMutex.unlock();

    // Get global frame
    std::string globalFrame = layered_costmap_->getGlobalFrameID();

    // Clear current points
    points.clear();

    // Get now
    ros::Time now = ros::Time::now();

    for (int index = 0; index < rangesCopy.size(); index++) {

        if (readingsTimeout > 0 && (now - rangesCopy[index].header.stamp).toSec() > readingsTimeout) {

            // Log
            ROS_ERROR_THROTTLE(1.0, "Range message timed out: %.2f", (now - rangesCopy[index].header.stamp).toSec());

            // Skip this range
            continue;

        }

        geometry_msgs::PointStamped pointInSensorFrame;
        geometry_msgs::PointStamped pointInGlobalFrame;

        // Set header in point
        pointInSensorFrame.header = rangesCopy[index].header;

        if (rangesCopy[index].min_range == rangesCopy[index].max_range) {

            if (std::isinf(rangesCopy[index].range)) {

                if (rangesCopy[index].range < 0) {

                    // Convert range to point stamped in sensor frame
                    pointInSensorFrame.point.x = rangesCopy[index].min_range;

                } else {

                    // Skip this range
                    continue;

                }

            } else {

                // Log
                ROS_ERROR_THROTTLE(1.0, "Range message with min_range == max_range must output -inf or +inf only");

                // Skip this range
                continue;

            }

        } else {

            if (rangesCopy[index].range >= rangesCopy[index].min_range && rangesCopy[index].range <= rangesCopy[index].max_range) {

                // Convert range to point stamped in sensor frame
                pointInSensorFrame.point.x = rangesCopy[index].range;

            } else {

                // Log
                ROS_ERROR_THROTTLE(1.0, "Range message with range %.2f out of %.2f - %.2f bounds", rangesCopy[index].range, rangesCopy[index].min_range, rangesCopy[index].max_range);

                // Skip this range
                continue;

            }

        }

        std::string errorMsg;

        if(tf_->waitForTransform(globalFrame, rangesCopy[index].header.frame_id, pointInSensorFrame.header.stamp, ros::Duration(tfTimeout), ros::Duration(0.01), & errorMsg) ) {

            // Transform point from sensor frame to global frame
            tf_->transformPoint(globalFrame, pointInSensorFrame, pointInGlobalFrame);

            points.push_back(pointInGlobalFrame);

        } else {

            // Log
            ROS_ERROR_THROTTLE(1.0, "Cannot transform from %s -> %s at %f: %s", globalFrame.c_str(), rangesCopy[index].header.frame_id.c_str(), rangesCopy[index].header.stamp.toSec(), errorMsg.c_str());

            // Skip this range
            continue;

        }

    }

}

void RangeLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y) {

    // Check if layer is enabled
    if (!enabled_) {
        return;
    }

    // Update points
    updatePoints();

    for (int index = 0; index < points.size(); index++) {

        // Get point x and y
        double pointX = points[index].point.x;
        double pointY = points[index].point.y;

        // Update bounds
        *min_x = std::min(* min_x, pointX);
        *min_y = std::min(* min_y, pointY);
        *max_x = std::max(* max_x, pointX);
        *max_y = std::max(* max_y, pointY);

    }

}

void RangeLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {

    if (!enabled_) {
        return;
    }

    // Update points
    for (int index = 0; index < points.size(); index++) {

        // Get point x and y
        double pointX = points[index].point.x;
        double pointY = points[index].point.y;

        unsigned int mx;
        unsigned int my;

        // Convert point to map indexes
        if (master_grid.worldToMap(pointX, pointY, mx, my)){

            // Set cost
            master_grid.setCost(mx, my, LETHAL_OBSTACLE);

        }

    }

}

}
