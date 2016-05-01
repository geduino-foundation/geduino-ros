/*
 Battery.h
 
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

#ifndef _BATTERY_H_
#define _BATTERY_H_

class Battery {

public:

  // Create new battery voltage sensor attached to given pin with given conversion parameter.
  // The conversion parameter are used to convert raw voltage to battery voltage with formula:
  // Vbatt = Vraw * paramA + paramB.
  Battery(const unsigned int _pin, float _paramA, float _paramB) : pin(_pin), paramA(_paramA), paramB(_paramB) {
  };
   
  // Get volts in [V]
  void getVolts(float * volts);

private:

  // The pin wich the battery is connected to
  unsigned int pin;

  // The linear parameters for conversion
  float paramA;
  float paramB;

};

#endif
