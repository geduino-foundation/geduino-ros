/*
 Rate.cpp
 Copyright (C) 2014 Alessandro Francescon
 
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

#include <Arduino.h>
#include "Rate.h"

/* Public methods */

Rate::Rate(float frequency) {

  // Calculate time interval
  timeInterval = 1000 / frequency;

  // Get now
  long now = millis();

  // Schedule next
  scheduleNext(now);

}

boolean Rate::ellapsed() {

  long now = millis();

  if (now >= nextTimeInterval) {

    // Increase delay counters
    delayCounter.increase(now - nextTimeInterval);
    
    // Schedule next
    scheduleNext(now);

    return true;

  } 
  else {
    return false;
  }

}

void Rate::start() {
  
  // Set duration start time
  durationStartTime = millis();
  
}

void Rate::end() {
  
  long now = millis();
  
  // Increase duration counters
  durationCounter.increase(now - durationStartTime);
  
}

Counter& Rate::getDelayCounter() {
  return delayCounter;
}

Counter& Rate::getDurationCounter() {
  return durationCounter;
}

/* Private methods */

void Rate::scheduleNext(long now) {

    // Set next time interval from now
    nextTimeInterval = now + timeInterval;

}
  


