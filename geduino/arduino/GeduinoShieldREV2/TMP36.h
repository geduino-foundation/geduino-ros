/*
 TMP36.h
 
 Copyright (C) 2016 Alessandro Francescon
 
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

#ifndef _TMP36_H_
#define _TMP36_H_

#include "MedianFilter.h"

class TMP36 {

public:

  // Create new TMP36 sensor attached to given pin
  TMP36(const unsigned int _pin) : pin(_pin) {
  };
  
  // Get temperature in [C]
  void getTemperature(float * temperature);

private:

  // The pin wich TMP36 is connected to
  unsigned int pin;

  // The median filter
  MedianFilter filter;
  
};

#endif


