/*
 odometry.cpp

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

#include <odometry.h>

#define TWO_PI 6.28318530717959

typedef Eigen::Matrix<double, 2, 2> Matrix2x2;
typedef Eigen::Matrix<double, 3, 2> Matrix3x2;

void Odometry::updatePosition(double dl, double dr, double time) {

    // Calculate ds and dth
    double ds = 0.5 * (dr + dl);
    double dth = (dr - dl) / wb;

    // Calculate middle theta
    double mdth = pos(2) + 0.5 * dth;
    double sinmdth = sin(mdth);
    double cosmdth = cos(mdth);

    // Calculate dpos
    Vector3 dpos(ds * cosmdth, ds * sinmdth, dth);

    // Update position
    pos += dpos;

    // Make sure orientation is between [0, 2PI[
    while (pos(2) >= TWO_PI) {
       pos(2) -= TWO_PI;
    }
    while (pos(2) < 0) {
        pos(2) += TWO_PI;
    }

    Matrix2x2 sigmaD;
    Matrix3x3 nablaFp;
    Matrix3x2 nablaFrl;

    // Pre calculation
    double ds_2wb = ds / (2 * wb);

    // Compute sigmaD
    sigmaD << kr * std::abs(dr), 0,
            0, kl * std::abs(dl);

    // Compute nablaFp
    nablaFp << 1, 0, -1 * ds * sinmdth,
            0, 1, ds * cosmdth,
            0, 0, 1;

    // Compute nablaFrl
    nablaFrl << 0.5 * cosmdth - ds_2wb * sinmdth, 0.5 * cosmdth + ds_2wb * sinmdth,
            0.5 * sinmdth + ds_2wb * cosmdth, 0.5 * sinmdth - ds_2wb * cosmdth,
            1 / wb, -1 / wb;

    // Update position covariance
    posCov = nablaFp * posCov * nablaFp.transpose() + nablaFrl * sigmaD * nablaFrl.transpose();
}

void Odometry::updateVelocity(double dl, double dr, double time) {

    if (lastUpdateTime > 0) {

       // Add deltas to history
       drHistory[historyIndex] = dr;
       dlHistory[historyIndex] = dl;
       dtHistory[historyIndex] = time - lastUpdateTime;

       double drSum = 0;
       double dlSum = 0;
       double dtSum = 0;

       for (char index = 0; index < HISTORY_SIZE; index++) {

          drSum += drHistory[index];
          dlSum += dlHistory[index];
          dtSum += dtHistory[index];

       }

       if (dtSum > 0) {

          // Update velocity
          vel(0) = (drSum + dlSum) / (2 * dtSum);
          vel(2) = (drSum - dlSum) / (wb * dtSum);

          // Compute sigma
          Matrix2x2 sigma;
          sigma << kr * std::abs(drSum), 0,
                  0, kl * std::abs(dlSum);

          // Compute transformation
          Matrix3x2 transformation;
          transformation << 0.5, 0.5,
                  0, 0,
                  1 / wb, -1 / wb;

          // Update velocity covariance
          velCov = transformation * sigma * transformation.transpose();

       } else {

          // Set linear and angular velocity to zero (avoid by zero division)
          vel(0) = 0;
          vel(2) = 0;

          // Fill velocity covariance with zeroes
          velCov.fill(0);

       }

       // Increase history index
       if (++historyIndex >= HISTORY_SIZE) {

          // Reset history index
          historyIndex = 0;

       }

    }

    // Set last update time
    lastUpdateTime = time;

}

void Odometry::reset() {

   // Reset last update time
   lastUpdateTime = 0;

   // Reset position and velocity
   pos.fill(0);
   vel.fill(0);
   posCov.fill(0);
   velCov.fill(0);

   // Reset history
   historyIndex = 0;

   for (char index = 0; index < HISTORY_SIZE; index++) {

       drHistory[index] = 0;
       dlHistory[index] = 0;
       dtHistory[index] = 0;

   }

}
