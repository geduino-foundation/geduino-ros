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

void sum(tf::Matrix3x3 & m1, const tf::Matrix3x3 & m2) {

   // Sum m1 to m2
   m1[0] += m2[0];
   m1[1] += m2[1];
   m1[2] += m2[2];

}

tf::Matrix3x3 trans(tfScalar th) {

   // Create t matrix
   tf::Matrix3x3 t = tf::Matrix3x3::getIdentity();
   t[0].m_floats[0] = tfCos(th);
   t[0].m_floats[1] = -tfSin(th);
   t[1].m_floats[0] = -t[0].m_floats[1];
   t[1].m_floats[1] = t[0].m_floats[0];

   return t;

}

void lineCov(tf::Matrix3x3 & cov, tfScalar & wb, tfScalar & kl, tfScalar & kr, tfScalar & ds) {

   // Pre-calculate values
   tfScalar kl2 = tfPow(kl, 2);
   tfScalar kr2 = tfPow(kr, 2);
   tfScalar k2sum = kr2 + kl2;
   tfScalar k2dif = kr2 - kl2;
   tfScalar ds2 = tfPow(ds, 2);
   tfScalar wb2 = tfPow(wb, 2);

   // Create c matrix
   tf::Matrix3x3 c = tf::Matrix3x3::getIdentity();
   c[0].m_floats[0] = ds * k2sum / 4;
   c[0].m_floats[1] = ds2 * k2dif / (4 * wb);
   c[0].m_floats[2] = ds * k2dif / (2 * wb);
   c[1].m_floats[0] = c[0].m_floats[1];
   c[1].m_floats[1] = tfPow(ds, 3) * k2sum / (3 * wb2);
   c[1].m_floats[2] = ds2 * k2sum / (2 * wb2);
   c[2].m_floats[0] = c[0].m_floats[2];
   c[2].m_floats[1] = c[1].m_floats[2];
   c[2].m_floats[2] = ds * k2sum / wb2;

   // Create phi matrix
   tf::Matrix3x3 phi = tf::Matrix3x3::getIdentity();
   phi[1].m_floats[2] = ds;

   // Calculate cov
   cov = phi * cov * phi.transpose();
   sum(cov, c);

}

void arcCov(tf::Matrix3x3 & cov, tfScalar & wb, tfScalar & kl, tfScalar & kr, tfScalar & dl, tfScalar & dr) {

   // Pre-calculate values
   tfScalar r = wb * (dr + dl) / (2 * (dr - dl));
   tfScalar a = (dr - dl) / wb;
   tfScalar kRsr = tfFabs(wb * dr * tfPow(kr, 2) / (dr - dl));
   tfScalar kRsl = tfFabs(wb * dl * tfPow(kl, 2) / (dr - dl));
   tfScalar ksum = kRsr + kRsl;
   tfScalar kdif = kRsr - kRsl;
   tfScalar r2wb2 = tfPow(r, 2) / tfPow(wb, 2);
   tfScalar ksumr2wb2 = ksum * r2wb2;
   tfScalar ksumrwb2 = ksum * r / tfPow(wb, 2);
   tfScalar kdifrwb = kdif * r / wb;
   tfScalar kdifwb = kdif / wb;
   tfScalar sina = tfSin(a);
   tfScalar sin2a = tfSin(2 * a);
   tfScalar cosa = tfCos(a);
   tfScalar cos2a = tfCos(2 * a);

   // Create c matrix
   tf::Matrix3x3 c = tf::Matrix3x3::getIdentity();
   c[0].m_floats[0] = ksumr2wb2 * (1.5 * a - 2 * sina + 0.25 * sin2a) + 0.25 * ksum * (0.5 * a + 0.25 * sin2a) + kdifrwb * (-0.5 * a + sina - 0.25 * sin2a);
   c[0].m_floats[1] = ksum * (0.25 * r2wb2 * (3 - 4 * cosa + cos2a) + 0.0625 * (-1 + cos2a)) + 0.25 * kdifrwb * (-1 + 2 * cosa - cos2a);
   c[0].m_floats[2] = ksumrwb2 * (a - sina) + 0.25 * kdifwb * sina;
   c[1].m_floats[0] = c[0].m_floats[1];
   c[1].m_floats[1] = (0.5 * a - 0.25 * sin2a) * (ksum * (r2wb2 + 0.25) - kdifrwb);
   c[1].m_floats[2] = (ksumrwb2 - 2 * kdifwb) * (1 - cosa);
   c[2].m_floats[0] = c[0].m_floats[2];
   c[2].m_floats[1] = c[1].m_floats[2];
   c[2].m_floats[2] = ksum * a / tfPow(wb, 2);

   if (a < 0) {

      // Get the negative of c
      c[0] *= -1;
      c[1] *= -1;
      c[2] *= -1;

   }

   // Create phi matrix
   tf::Matrix3x3 phi = tf::Matrix3x3::getIdentity();
   phi[0].m_floats[0] = cosa;
   phi[0].m_floats[1] = sina;
   phi[0].m_floats[2] = r * (1 - cosa);
   phi[1].m_floats[0] = -sina;
   phi[1].m_floats[1] = cosa;
   phi[1].m_floats[2] = r * sina;

   // Calculate arcCov
   cov = phi * cov * phi.transpose();
   sum(cov, c);

}

void Odometry::update(tfScalar dl, tfScalar dr, tfScalar time) {

   // Calculate ds and dth
   tfScalar ds = 0.5 * (dr + dl);
   tfScalar dth = (dr - dl) / wb;

   // Calculate middle theta
   tfScalar mdth = angPos.z() + 0.5 * dth;

   // Calculate dlinpos and dangpos
   tf::Vector3 dlinpos(ds * cos(mdth), ds * sin(mdth), 0);
   tf::Vector3 dangpos(0, 0, dth);

   // Update position
   linPos += dlinpos;
   angPos += dangpos;

   // Make sure orientation is between [0, 2PI[
   while (angPos.z() >= TWO_PI) {
      angPos.m_floats[2] -= TWO_PI;
   }
   while (angPos.z() < 0) {
      angPos.m_floats[2] += TWO_PI;
   }

   // Get path radius
   tfScalar r = wb * (dr + dl) / (2 * (dr - dl));


   if (tfFabs(r) < rTres) {

      // Update position covariance as constant radius arc path
      arcCov(cov, wb, kl, kr, dl, dr);

   } else {

      // Update position covariance as straight path
      lineCov(cov, wb, kl, kr, ds);

   }

   if (lastUpdateTime > 0) {

      // Add deltas to history
      dsHistory[historyIndex] = ds;
      dthHistory[historyIndex] = dth;
      dtHistory[historyIndex] = time - lastUpdateTime;

      tfScalar dsSum = 0;
      tfScalar dthSum = 0;
      tfScalar dtSum = 0;

      for (char index = 0; index < HISTORY_SIZE; index++) {

         dsSum += dsHistory[index];
         dthSum += dthHistory[index];
         dtSum += dtHistory[index];

      }

      if (dtSum > 0) {

         // Calculate linear and angular velocity
         linVel.m_floats[0] = dsSum / dtSum;
         angVel.m_floats[2] = dthSum / dtSum;

      } else {

         // Set linear and angular velocity to zero (avoid by zero division)
         linVel.m_floats[0] = 0;
         angVel.m_floats[2] = 0;

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

void Odometry::getPosition(tf::Vector3 & linear, tf::Vector3 & angular) {
   linear = linPos;
   angular = angPos;
}

void Odometry::getVelocity(tf::Vector3 & linear, tf::Vector3 & angular) {
   linear = linVel;
   angular = angVel;
}

void Odometry::getCovariance(tf::Matrix3x3 & covariance) {

   // Apply transformation from local robot frame to world reference frame
   tf::Matrix3x3 t = trans(-angPos.z());
   covariance = t * cov * t.transpose();

}

void Odometry::reset() {

   // Reset last update time
   lastUpdateTime = 0;

   // Reset linear and angular position
   linPos.setValue(0, 0, 0);
   angPos.setValue(0, 0, 0);

   // Reset history
   historyIndex = 0;

   for (char index = 0; index < HISTORY_SIZE; index++) {

       dsHistory[index] = 0;
       dthHistory[index] = 0;
       dtHistory[index] = 0;

   }

   // Reset linear and angular velocity
   linVel.setValue(0, 0, 0);
   angVel.setValue(0, 0, 0);

   // Reset covariance
   cov.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);

}
