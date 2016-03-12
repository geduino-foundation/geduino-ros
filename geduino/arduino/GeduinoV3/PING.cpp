/*
 PING.cpp
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
 
#include <ctype.h>
#include <Arduino.h>
#include "PING.h"

PING::PING(const unsigned int _pin) {
  
  // Set pin
  pin = _pin;
  
}

const float PING::measure(const float temperature) {
  
  // Measure duration
  const unsigned long duration = measure_duration();

  if (duration == 0) {

    // Increase failure by one
    failures.increase(1);
    
    // Wrong measurement
    return -1;

  } 
  else {
    
    // Increase failure by zerp, i.e. no failure
    failures.increase(0);

    // Convert duration to m
    const float measure = duration_to_m(duration, temperature);

    return measure;

  }

}

Counter& PING::getFailureCounter() {
  return failures;
}

const unsigned long PING::measure_duration() {

  // Set pin to output
  pinMode(pin,  OUTPUT);

  // Send clean low pulse
  digitalWrite(pin, LOW);
  delayMicroseconds(2);

  // Send start pulse
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // Set pin to input
  pinMode(pin, INPUT);

  // Read measure pulse duration
  const unsigned long duration = pulseIn(pin, HIGH);

  return duration;

}

const float PING::duration_to_m(const unsigned long duration, const float temperature) {

  // Calculate microsencods per m
  const float microseconds_per_m = 1000000 / (331.5 + (0.6 * temperature));

  // Calculate sensor offset
  const float sensor_offset = MOUNTING_GAP * microseconds_per_m;

  // Get net duration removing time result by mounting gap
  const float net_duration = max(0, duration - sensor_offset);

  // Calculate m
  const float m = net_duration / microseconds_per_m / 2;

  return m;

}
