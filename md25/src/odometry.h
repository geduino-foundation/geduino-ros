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
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

/*
 Implaments odometry for a differential drive robot. Position covariance model
 is provided by Lindsay KLEEMAN from Monash University, Technical Report MECSE-95-1-1995.
 */
class Odometry {

   public:

      // Create odometry for a DDR with given wheel base and error constants
      Odometry(tfScalar _wb, tfScalar _kl, tfScalar _kr, tfScalar _rTres) : wb(_wb), kl(_kl), kr(_kr), rTres(_rTres) {
         reset();
      };

      // Update position and covariance for given wheel path increments and time
      void update(tfScalar dl, tfScalar dr, tfScalar time);

      // Get linear and angular position vector in world reference frame
      void getPosition(tf::Vector3 & linear, tf::Vector3 & angular);

      // Get linear and angular velocity in world reference frame
      void getVelocity(tf::Vector3 & linear, tf::Vector3 & angular);

      // Get position covariance matrix in world reference frame
      void getCovariance(tf::Matrix3x3 & covariance);

      // Reset position and covariance
      void reset();

   private:

      // The robot wheel base
      tfScalar wb;

      // The error constants
      tfScalar kl, kr;

      // The threshold radius. The path will be considered straight if radius is greater than given threshold.
      tfScalar rTres;

      // The last update time
      tfScalar lastUpdateTime;

      // The linear and angular position vector in world reference frame
      tf::Vector3 linPos, angPos;

      // The linear and angular velocity in robot reference frame
      tf::Vector3 linVel, angVel;

      // The position covariance matrix in local robot reference frame
      tf::Matrix3x3 cov;

};

#endif
