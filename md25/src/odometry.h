/*
 odometry.h

 Copyright (C) 2015 Alessandro Francescon
 
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


#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <math.h>
#include <Eigen/Core>

#define HISTORY_SIZE 3

typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3;

/*
 Implements odometry for a differential drive robot
 */
class Odometry {

   public:

      // Create odometry for a DDR with given wheel base and error constants
      Odometry(double _wb, double _kl, double _kr) : wb(_wb), kl(_kl), kr(_kr) {
         reset();
      };

      // Update position and covariance for given wheel path increments and time
      void update(double dl, double dr, double time) {

          // Update position
          updatePosition(dl, dr, time);

          // Update velocity
          updateVelocity(dl, dr, time);

      };

      // Get position vector as (x, y, th) and its covariance matrix in odom reference frame
      void getPosition(Vector3 & _pos, Matrix3x3 & _posCov) {
          _pos = pos;
          _posCov = posCov;
      };

      // Get velocity vector as (vx, vy, vth) and its covariance matrix in robot reference frame
      void getVelocity(Vector3 & _vel, Matrix3x3 & _velCov) {
          _vel = vel;
          _velCov = velCov;
      };

      // Reset position and covariance
      void reset();

   private:

      // The robot wheel base
      double wb;

      // The error constants
      double kl, kr;

      // The last update time
      double lastUpdateTime;

      // The position and velocity vector
      Vector3 pos, vel;

      // The position and velocity covariance matrix
      Matrix3x3 posCov, velCov;

      // The history index and arrays
      char historyIndex;
      double drHistory[HISTORY_SIZE], dlHistory[HISTORY_SIZE], dtHistory[HISTORY_SIZE];

      // Update position and its covariance
      void updatePosition(double dl, double dr, double time);

      // Update velocity and its covariance
      void updateVelocity(double dl, double dr, double time);

};

#endif
