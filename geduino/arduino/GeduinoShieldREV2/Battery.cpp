/*
 Battery.cpp

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
 
#include <ctype.h>
#include <Arduino.h>
#include "Battery.h"

#define ANALOG_REFERENCE 3.3
#define ANALOG_RESOLUTION 1024

void Battery::getVolts(float * volts) {
  
  // Get raw volts in [V]
  float raw = analogRead(pin) * ANALOG_REFERENCE / ANALOG_RESOLUTION * paramA + paramB;
  
  // Filter
  filter.filter(raw, volts);  
  
}
