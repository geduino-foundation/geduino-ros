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
#include <rolling_window.h>
#include <Eigen/Core>

typedef Eigen::Matrix<double, 3, 1> Vector3;

/*
 Implements odometry for a differential drive robot
 */
class Odometry {

   public:

      // Create odometry for a DDR with given wheel base
      Odometry(double _wb, char _rollingWindowSize) :
            wb(_wb),
            drRollingWindow(RollingWindow(_rollingWindowSize)),
            dlRollingWindow(RollingWindow(_rollingWindowSize)),
            dtRollingWindow(RollingWindow(_rollingWindowSize)) {

          reset();

      };

      // Update position and covariance for given wheel path increments and time
      void update(double dl, double dr, double time) {

          // Update position
          updatePosition(dl, dr);

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

      // The rolling windows used to compute velocity
      RollingWindow drRollingWindow;
      RollingWindow dlRollingWindow;
      RollingWindow dtRollingWindow;

      // Update position
      void updatePosition(double dl, double dr);

      // Update velocity
      void updateVelocity(double dl, double dr, double time);

};

#endif
