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

void Odometry::updatePosition(double dl, double dr) {

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

}

void Odometry::updateVelocity(double dl, double dr, double time) {
    
    if (lastUpdateTime > 0) {

        // Add values to rolling window
        drRollingWindow.add(dr);
        dlRollingWindow.add(dl);
        dtRollingWindow.add(time - lastUpdateTime);

        // Get sums
        double drSum;
        double dlSum;
        double dtSum;
        drRollingWindow.sum(& drSum);
        dlRollingWindow.sum(& dlSum);
        dtRollingWindow.sum(& dtSum);

        if (dtSum > 0) {
            
            // Update velocity
            vel(0) = (drSum + dlSum) / (2 * dtSum);
            vel(2) = (drSum - dlSum) / (wb * dtSum);
            
        } else {
            
            // Set linear and angular velocity to zero (avoid by zero division)
            vel(0) = 0;
            vel(2) = 0;
            
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

   // Reset rolling window
   drRollingWindow.reset();
   dlRollingWindow.reset();
   dtRollingWindow.reset();

}
