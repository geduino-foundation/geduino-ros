/*
 Counter.h
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
 
#ifndef _COUNTER_H_
#define _COUNTER_H_

#include <ctype.h>
#include <Arduino.h>

class Counter {

public:

  // Create new counter
  Counter();
  
  // Increase counter sum by given number: invoking this method
  // will trigger increase of counter by one.
  void increase(unsigned long num);
  
  // Get counter sun and count and reset them
  void getCounters(unsigned long * _sum, unsigned long  * _counter);

private:

  unsigned long sum;
  unsigned long counter;

};

#endif
