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

/*
 Implements odometry for a differential drive robot
 */
class Odometry {

   public:

      // Create odometry for a DDR with given wheel base
      Odometry(double _wb) : wb(_wb) {
         reset();
      };

      // Update position and covariance for given wheel path increments and time
      void update(double dl, double dr, double time) {

          // Update position
          updatePosition(dl, dr, time);

          // Update velocity
          updateVelocity(dl, dr, time);

      };

      // Get position vector as (x, y, th)
      void getPosition(Vector3 & _pos) {
          _pos = pos;
      };

      // Get velocity vector as (vx, vy, vth)
      void getVelocity(Vector3 & _vel) {
          _vel = vel;
      };

      // Reset position and covariance
      void reset();

   private:

      // The robot wheel base
      double wb;

      // The last update time
      double lastUpdateTime;

      // The position and velocity vector
      Vector3 pos, vel;

      // The history index and arrays
      char historyIndex;
      double drHistory[HISTORY_SIZE], dlHistory[HISTORY_SIZE], dtHistory[HISTORY_SIZE];

      // Update position and its covariance
      void updatePosition(double dl, double dr, double time);

      // Update velocity and its covariance
      void updateVelocity(double dl, double dr, double time);

};

#endif
