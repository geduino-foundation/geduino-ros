/*
 geometry_conversion.cpp

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

#ifndef GEOMETRY_CONVERSION
#define GEOMETRY_CONVERSION

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <md25/Euler.h>

void toEuler(const geometry_msgs::Quaternion & quaternion, md25::Euler * euler) {

    // Get quaternion
    tf::Quaternion q(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w);

    // Put into matrix
    tf::Matrix3x3 matrix(q);

    // Get RPY
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    // Set to euler
    euler->roll = roll;
    euler->pitch = pitch;
    euler->yaw = yaw;

}

#endif

