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

#define MOUNTING_GAP 0.002
#define DEFAULT_PIN 7

class PING {

public:

  // Create new PING sensor attached to given pin
  PING(const unsigned int pin);

  // Get distance measure in m. Measure is corrected for given temperature (in Celsius degree).
  // If wrong measure is detected (for ex. no pulse duration from PING))) ) it will return -1.
  // The measure is corrected to take count of the mounting gap.
  const float measure(const float temperature);

  // Get the failure counter
  Counter& getFailureCounter();

private:
 
  Counter failures;

  // The ping wich PING))) is connected to
  unsigned int pin;

  // Start new measure and return the measure pulse duration in microseconds
  const unsigned long measure_duration();

  // Convert duration to m with temperature correction
  const float duration_to_m(const unsigned long duration, const float temperature);

};

#endif



