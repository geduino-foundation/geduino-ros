/*
 tf2_imu_message.cpp

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

#ifndef TF2_IMU_MESSAGE
#define TF2_IMU_MESSAGE

#include <tf2/convert.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace tf2 {

    template <> inline const ros::Time & getTimestamp(const sensor_msgs::Imu & imu) {return imu.header.stamp;}

    template <> inline const std::string & getFrameId(const sensor_msgs::Imu & imu) {return imu.header.frame_id;}

    inline sensor_msgs::Imu toMsg(const sensor_msgs::Imu & imu) { return imu; }

    inline void fromMsg(const sensor_msgs::Imu & imu_in, sensor_msgs::Imu & imu_out) { imu_out = imu_in; }

    inline void transformCovariance(const boost::array<double, 9> & in, boost::array<double, 9> & out, Eigen::Quaternion<double> r){

        // Convert covariance vector into matrix
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_in(in.data());
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_out(out.data());

        // Transform covariance
        cov_out = r * cov_in * r.inverse();

    }

    template <> inline void doTransform(const sensor_msgs::Imu &imu_in, sensor_msgs::Imu &imu_out, const geometry_msgs::TransformStamped& t_in) {

        imu_out.header = t_in.header;

        // Discard translation, only use orientation for IMU transform
        Eigen::Quaternion<double> r(
                    t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
        Eigen::Transform<double,3,Eigen::Affine> t(r);

        // Transform angular velocity
        Eigen::Vector3d angular_velocity = t * Eigen::Vector3d(
                    imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

        imu_out.angular_velocity.x = angular_velocity.x();
        imu_out.angular_velocity.y = angular_velocity.y();
        imu_out.angular_velocity.z = angular_velocity.z();

        // Transform angular velocity covariance
        transformCovariance(imu_in.angular_velocity_covariance, imu_out.angular_velocity_covariance, r);

        // Transform linear acceleration
        Eigen::Vector3d linear_acceleration = t * Eigen::Vector3d(
                    imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);

        imu_out.linear_acceleration.x = linear_acceleration.x();
        imu_out.linear_acceleration.y = linear_acceleration.y();
        imu_out.linear_acceleration.z = linear_acceleration.z();

        // Transform linear acceleration covariance
        transformCovariance(imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, r);

        // Transform orientation
        Eigen::Quaternion<double> orientation = r * Eigen::Quaternion<double>(
                    imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z) * r.inverse();

        imu_out.orientation.w = orientation.w();
        imu_out.orientation.x = orientation.x();
        imu_out.orientation.y = orientation.y();
        imu_out.orientation.z = orientation.z();

        // Transform orientation covariance
        transformCovariance(imu_in.orientation_covariance, imu_out.orientation_covariance, r);

    }

}

#endif

