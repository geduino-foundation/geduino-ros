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

void Odometry::updatePosition(double dl, double dr, double angularVelocityZ, double dt) {

    // Calculate ds, dth and th
    double ds = 0.5 * (dr + dl);
    double dth = (dr - dl) / wb;
    double th = pos(2) + dth;

    // Compute odometry
    computeOdometry(pos, ds, th);;

    if (posFilterTau > 0 && dt > 0) {

        // Compute filter parameter for current cycle
        double filterA = posFilterTau / (posFilterTau + dt);

        // Filter th
        double filterTh = filterA * (filteredPos(2) + angularVelocityZ * dt) + (1 - filterA) * pos(2);

        // Compute odometry
        computeOdometry(filteredPos, ds, filterTh);

    } else {

        // Filter not applied: set filtered position as position
        filteredPos = pos;

    }

}

void Odometry::updateVelocity(double dl, double dr, double angularVelocityZ, double dt) {
    
    if (dt > 0) {

        // Add values to rolling window
        drRollingWindow.add(dr);
        dlRollingWindow.add(dl);
        dtRollingWindow.add(dt);

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

    if (velFilterTau > 0) {

        // Filter velocity
        filteredVel(0) = vel(0);
        filteredVel(1) = vel(1);
        filteredVel(2) = velFilterTau * angularVelocityZ + (1 - velFilterTau) * vel(2);

    } else {

        // Filter not applied: set filtered velocity as velocity
        filteredVel = vel;

    }
    
}

void Odometry::reset() {

   // Reset last update time
   lastUpdateTime = 0;

   // Reset position and velocity
   pos.fill(0);
   filteredPos.fill(0);
   vel.fill(0);
   filteredVel.fill(0);

   // Reset rolling window
   drRollingWindow.reset();
   dlRollingWindow.reset();
   dtRollingWindow.reset();

}

void Odometry::computeOdometry(Vector3 & _pos, double ds, double th) {

    // Calculate middle theta
    double mdth = 0.5 * (_pos(2) + th);
    double sinmdth = sin(mdth);
    double cosmdth = cos(mdth);

    // Update position
    _pos(0) += ds * cosmdth;
    _pos(1) += ds * sinmdth;
    _pos(2) = th;

    // Make sure orientation is between [0, 2PI[
    constrainPosition(_pos);

}

void Odometry::constrainPosition(Vector3 & _pos) {

    // Make sure orientation is between [0, 2PI[
    while (_pos(2) >= TWO_PI) {
        _pos(2) -= TWO_PI;
    }
    while (_pos(2) < 0) {
        _pos(2) += TWO_PI;
    }

}
