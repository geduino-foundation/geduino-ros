/*
 complementary_filter_node.cpp

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

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_imu_message.h>
#include <geometry_conversion.h>

// The odometry and IMU sync policy
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> OdomImuSyncPolicy;

// The pointer to odometry message publisher
ros::Publisher * odometryMessagePublisherPtr;

// The pointer tp odometry transform broadcaster
tf::TransformBroadcaster * odometryTransformBroadcasterPtr;

// The transform listener
tf2_ros::Buffer * transformBufferPtr;
tf2_ros::TransformListener * transformListenerPtr;

// Pose and velocity covariance
std::vector<double> posCovarianceDiagonal;
std::vector<double> velCovarianceDiagonal;

// Complementary filter parameters
float filter_a, filter_tau;

// The filter last update time
double lastUpdateTime;

// The filtered orientation
double filtered_orientation_z;

// Node parameters
bool publishOdomTransformation;

void callback(const nav_msgs::OdometryConstPtr & odometry, const sensor_msgs::ImuConstPtr & imu) {

    // Get now (currently use odometry time...)
    ros::Time now = odometry->header.stamp;

    // Convert quaternion to euler
    md25::Euler euler;
    toEuler(odometry->pose.pose.orientation, & euler);

    // Filtered angular velocity z
    double filtered_angular_velocity_z;

    if (lastUpdateTime == 0) {

        // Log
        ROS_INFO("Received first combined odometry and imu message: initializing filter...");

        // Set initial orientation from odometry
        filtered_orientation_z = euler.yaw;

        // Set filtered angular velocity to odometry velocity
        filtered_angular_velocity_z = odometry->twist.twist.angular.z;

    } else {

        // Transform imu to base link frame
        sensor_msgs::Imu transformedImu;

        std::string errorMsg;

        if (transformBufferPtr->canTransform(odometry->child_frame_id, imu->header.frame_id, imu->header.stamp, ros::Duration(0.01), & errorMsg)) {

            // Transform imu message
            transformBufferPtr->transform(* imu, transformedImu, odometry->child_frame_id);

        } else {

            // Log
            ROS_ERROR_THROTTLE(1.0, "Cannot transform from %s -> %s at %f: %s", imu->header.frame_id.c_str(), odometry->child_frame_id.c_str(), imu->header.stamp.toSec(), errorMsg.c_str());

            return;

        }

        // Calculate delta time
        double dt = now.toSec() - lastUpdateTime;

        // Update a
        filter_a = filter_tau / (filter_tau + dt);

        // Filter orientation
        filtered_orientation_z = filter_a * (filtered_orientation_z + transformedImu.angular_velocity.z * dt) + (1 - filter_a) * euler.yaw;

        // Filter angular velocity
        filtered_angular_velocity_z = filter_tau * transformedImu.angular_velocity.z + (1 - filter_tau) * odometry->twist.twist.angular.z;

    }

    // Create odometry quaternion from filtered orientation
    geometry_msgs::Quaternion filteredOdometryQuaternion = tf::createQuaternionMsgFromYaw(filtered_orientation_z);

    // Create odometry message
    nav_msgs::Odometry odometryMessage;
    odometryMessage.header.frame_id = odometry->header.frame_id;
    odometryMessage.child_frame_id = odometry->child_frame_id;
    odometryMessage.header.stamp = now;
    odometryMessage.pose.pose.position.x = odometry->pose.pose.position.x;
    odometryMessage.pose.pose.position.y = odometry->pose.pose.position.y;
    odometryMessage.pose.pose.position.z = odometry->pose.pose.position.z;
    odometryMessage.pose.pose.orientation = filteredOdometryQuaternion;
    odometryMessage.twist.twist.linear.x = odometry->twist.twist.linear.x;
    odometryMessage.twist.twist.linear.y = odometry->twist.twist.linear.y;
    odometryMessage.twist.twist.angular.z = odometry->twist.twist.linear.z;
    odometryMessage.twist.twist.angular.x = odometry->twist.twist.angular.x;
    odometryMessage.twist.twist.angular.y = odometry->twist.twist.angular.y;
    odometryMessage.twist.twist.angular.z = filtered_angular_velocity_z;
    odometryMessage.pose.covariance[0] = posCovarianceDiagonal[0];
    odometryMessage.pose.covariance[7] = posCovarianceDiagonal[1];
    odometryMessage.pose.covariance[14] = posCovarianceDiagonal[2];
    odometryMessage.pose.covariance[21] = posCovarianceDiagonal[3];
    odometryMessage.pose.covariance[28] = posCovarianceDiagonal[4];
    odometryMessage.pose.covariance[35] = posCovarianceDiagonal[5];
    odometryMessage.twist.covariance[0] = velCovarianceDiagonal[0];
    odometryMessage.twist.covariance[7] = velCovarianceDiagonal[1];
    odometryMessage.twist.covariance[14] = velCovarianceDiagonal[2];
    odometryMessage.twist.covariance[21] = velCovarianceDiagonal[3];
    odometryMessage.twist.covariance[28] = velCovarianceDiagonal[4];
    odometryMessage.twist.covariance[35] = velCovarianceDiagonal[5];

    // Publish filtered odometry
    odometryMessagePublisherPtr->publish(odometryMessage);

    if (publishOdomTransformation) {

        // Create odometry transformation
        geometry_msgs::TransformStamped odometryTransformation;
        odometryTransformation.header.stamp = now;
        odometryTransformation.header.frame_id = odometry->header.frame_id;
        odometryTransformation.child_frame_id = odometry->child_frame_id;
        odometryTransformation.transform.translation.x = odometry->pose.pose.position.x;
        odometryTransformation.transform.translation.y = odometry->pose.pose.position.y;
        odometryTransformation.transform.translation.z = odometry->pose.pose.position.z;
        odometryTransformation.transform.rotation = filteredOdometryQuaternion;

        // Broadcast odometry transformation
        odometryTransformBroadcasterPtr->sendTransform(odometryTransformation);

    }

    // Update last update time
    lastUpdateTime = now.toSec();

}

int main(int argc, char** argv) {

    // Init ros
    ros::init(argc, argv, "complementary_filtrer_node");

    // Get node handle
    ros::NodeHandle nodeHandle;

    // Get private node handle
    ros::NodeHandle privateNodeHandle("~");

    // Get parameters
    privateNodeHandle.param("filter_tau", filter_tau, 0.075f);
    privateNodeHandle.getParam("pos_covariance_diagonal", posCovarianceDiagonal);
    privateNodeHandle.getParam("vel_covariance_diagonal", velCovarianceDiagonal);
    privateNodeHandle.param("publish_odom_transformation", publishOdomTransformation, true);

    if (posCovarianceDiagonal.size() != 6) {

        // Log
        ROS_WARN("pos_covariance_diagonal size must be 6, actual: %lu. Use zeroes.", posCovarianceDiagonal.size());

        // Assign a six sized vector fill with zeroes
        posCovarianceDiagonal.assign(6, 0.0f);

    }

    if (velCovarianceDiagonal.size() != 6) {

        // Log
        ROS_WARN("vel_covariance_diagonal size must be 6, actual: %lu. Use zeroes.", velCovarianceDiagonal.size());

        // Assign a six sized vector fill with zeroes
        velCovarianceDiagonal.assign(6, 0.0f);

    }

    // Log
    ROS_INFO("complementary filter tau: %g", filter_tau);

    // Create subscribers
    message_filters::Subscriber<nav_msgs::Odometry> odometrySubscriber(nodeHandle, "/odom", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imuSubscriber(nodeHandle, "/imu", 1);

    // Synchronize subscribers
    message_filters::Synchronizer<OdomImuSyncPolicy> timeSynchronizer(OdomImuSyncPolicy(10), odometrySubscriber, imuSubscriber);
    timeSynchronizer.registerCallback(boost::bind(& callback, _1, _2));

    // Create odometry message publisher
    ros::Publisher odometryMessagePublisher = nodeHandle.advertise<nav_msgs::Odometry>("/odom_filtered", 20);
    odometryMessagePublisherPtr = & odometryMessagePublisher;

    // The transform listener
    tf2_ros::Buffer transformBuffer;
    tf2_ros::TransformListener transformListener(transformBuffer);
    transformBufferPtr = & transformBuffer;
    transformListenerPtr = & transformListener;

    // The odometry transform broadcaster
    tf::TransformBroadcaster odometryTransformBroadcaster;
    odometryTransformBroadcasterPtr = & odometryTransformBroadcaster;


    // Init filter parameters
    filter_a = 0;

    // Init last update time
    lastUpdateTime = 0;

    // Spin
    ros::spin();

    return 0;

}
