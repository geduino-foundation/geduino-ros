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

int PING::measure(float temperature, float * measurement) {
  
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
  long duration = pulseIn(pin, HIGH);
  
  if (duration > 0) {
    
    // Increase failure by zerp, i.e. no failure
    failures.increase(0);
    
    // Calculate sound speed
    float soundSpeed = 331.3 + 0.6 * temperature;
    
    // Transform duration to measurement in [m]
    *measurement = max(0, (duration * soundSpeed / 1000000 - mountingGap) / 2);
    
  } else {
    
    // Increase failure by one
    failures.increase(1);
    
    // Measurement failed
    return -1;
    
  }

}

Counter& PING::getFailureCounter() {
  return failures;
}
