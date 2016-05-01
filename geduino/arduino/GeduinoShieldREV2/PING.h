/*
 PING.h
 Implementation based on technical specification available at:
 http://www.parallax.com/
 
 Copyright (C) 2013 Alessandro Francescon
 
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

#ifndef _PING_H_
#define _PING_H_

#include "Counter.h"

class PING {

public:

  // Create new PING))) sensor attached to given pin and mounting gap
  PING(const unsigned int _pin, float _mountingGap) : pin(_pin), mountingGap(_mountingGap) {
  };

  // Start a measure on this PING))) sensor. This method measure the distance from PING)))
  // sensor in m. The temperature is used to calibrate sound speed.
  // Return -1 if measurement fails, 0 if success.
  int measure(float temperature, float * measurement);

  // Get the failure counter
  Counter& getFailureCounter();

private:

  // The failure counter
  Counter failures;

  // The pin wich PING))) is connected to
  unsigned int pin;
  
  // The PING))) mounting gap in [m]
  float mountingGap;

};

#endif



