#ifndef RANGE_LAYER_H_
#define RANGE_LAYER_H_

/*
 range_layer.h
    
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

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PointStamped.h>
#include <range_layer/RangeLayerConfig.h>

namespace range_layer {

class RangeLayer : public costmap_2d::Layer {

public:

    RangeLayer();

    virtual void onInitialize();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
                              double* max_y);

    virtual void updateCosts(costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

    void rangeCallback(const sensor_msgs::Range & rangeMsg, const std::string & rangeTopic);

private:

    void reconfigureCallback(range_layer::RangeLayerConfig & config, uint32_t level);

    void updatePoints();

    double mark_x_, mark_y_;

    dynamic_reconfigure::Server<RangeLayerConfig> * dynamicReconfigureServer;

    std::vector<ros::Subscriber> rangeSubscribers;

    boost::mutex rangesMutex;

    std::map<std::string, sensor_msgs::Range> ranges;

    std::vector<geometry_msgs::PointStamped> points;

    double readingsTimeout, tfTimeout;

};

}

#endif
