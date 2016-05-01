/*
 TMP36.cpp

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
#include "TMP36.h"

#define ANALOG_REFERENCE 3.3
#define ANALOG_RESOLUTION 1024

void TMP36::getTemperature(float * temperature) {

  // Get temperature in dK
  * temperature = analogRead(pin) * ANALOG_REFERENCE * 100 / ANALOG_RESOLUTION + 223.15;

}
