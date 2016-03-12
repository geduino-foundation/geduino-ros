/*
 Rate.h
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
 
#ifndef _RATE_H_
#define _RATE_H_

#include <ctype.h>
#include <Arduino.h>
#include "Counter.h"

class Rate {

public:

  // Create new rate with given frequency
  Rate(float frequency);
  
  // Get if current rate is ellapsed and, if it is, schedule next
  boolean ellapsed();
  
  // Start measurement of duration
  void start();
  
  // End measurement of duration
  void end();
  
  // Get delay counter of this rate
  Counter& getDelayCounter();

  // Get duration counter
  Counter& getDurationCounter();

private:

  unsigned long timeInterval;
  unsigned long nextTimeInterval;
  unsigned long durationStartTime;
  
  Counter delayCounter;
  Counter durationCounter;

  // Schedule next rate starting from now
  void scheduleNext(long now);

};

#endif
