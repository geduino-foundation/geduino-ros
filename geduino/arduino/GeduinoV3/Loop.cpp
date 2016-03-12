/*
 Loop.cpp
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
#include "Loop.h"

/* Public methods */

Loop::Loop() {
}

void Loop::cycleUsed() {

  // Set used
  used = true;

}

void Loop::cyclePerformed() {
  
  if (used) {
    
    // Increase used cycles by one
    usedCycles.increase(1);
    
  }
  else {
    
    // Increase used cycles by zero, i.e. unused cycle
    usedCycles.increase(0);
    
  }
  
  // Reset used
  used = false;
  
}

Counter& Loop::getUsedCounter() {
  return usedCycles;
}


