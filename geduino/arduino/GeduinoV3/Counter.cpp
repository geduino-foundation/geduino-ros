/*
 Counter.cpp
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
#include "Counter.h"

/* Public methods */

Counter::Counter() {

  // Set initial counters
  sum = 0;
  counter = 0;

}

void Counter::increase(unsigned long num) {

  // Increase sum and counter
  sum += num;
  counter++;

}

void Counter::getCounters(unsigned long * _sum, unsigned long  * _counter) {

  // Set counters
  *_sum = sum;
  *_counter = counter;

  // Reset counters
  sum = 0;
  counter = 0;

}


