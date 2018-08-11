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
      Odometry(double _wb, char _rollingWindowSize, double _posFilterTau, double _velFilterTau) :
            wb(_wb),
            drRollingWindow(RollingWindow(_rollingWindowSize)),
            dlRollingWindow(RollingWindow(_rollingWindowSize)),
            dtRollingWindow(RollingWindow(_rollingWindowSize)),
            posFilterTau(_posFilterTau),
            velFilterTau(_velFilterTau) {

          reset();

      };

      // Update position and covariance for given wheel path increments and time filtered by IMU yaw velocity.
      // If you not want to filter position set tau filter to 0 and angularVelocityZ will be ignored.
      void update(double dl, double dr, double angularVelocityZ, double time) {

          double dt = 0;

          if (lastUpdateTime > 0) {

              // Calculate dt
              dt = time - lastUpdateTime;

          }

          // Update position
          updatePosition(dl, dr, angularVelocityZ, dt);

          // Update velocity
          updateVelocity(dl, dr, angularVelocityZ, dt);

          // Set last update time
          lastUpdateTime = time;

      };

      // Get position vector as (x, y, th)
      void getPosition(Vector3 & _pos) {
          _pos = pos;
      };

      // Get filtered position vector as (x, y, th)
      void getFilteredPosition(Vector3 & _filteredPos) {
          _filteredPos = filteredPos;
      };

      // Get velocity vector as (vx, vy, vth)
      void getVelocity(Vector3 & _vel) {
          _vel = vel;
      };

      // Get filtered velocity vector as (vx, vy, vth)
      void getFilteredVelocity(Vector3 & _filteredVel) {
          _filteredVel = filteredVel;
      };

      // Reset position and covariance
      void reset();

   private:

      // The robot wheel base
      double wb;

      // The last update time
      double lastUpdateTime;

      // The position and velocity vector (raw and filtered)
      Vector3 pos, filteredPos, vel, filteredVel;

      // The filter tau to be used for position and velocity
      double posFilterTau, velFilterTau;

      // The rolling windows used to compute velocity
      RollingWindow drRollingWindow;
      RollingWindow dlRollingWindow;
      RollingWindow dtRollingWindow;

      // Update position: at first run dt will be 0, make sure implementation can halde this case
      void updatePosition(double dl, double dr, double angularVelocityZ, double dt);

      // Update velocity: at first run dt will be 0, make sure implementation can halde this case
      void updateVelocity(double dl, double dr, double angularVelocityZ, double dt);

      void computeOdometry(Vector3 & _pos, double ds, double th);

      // Constrain position orientation in [0, 2PI[ range
      void constrainPosition(Vector3 & _pos);

};

#endif
