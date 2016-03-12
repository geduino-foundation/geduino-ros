/*
 Loop.h
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
 
#ifndef _LOOP_H_
#define _LOOP_H_

#include <ctype.h>
#include <Arduino.h>
#include "Rate.h"

class Loop {

public:

  // Create new loop
  Loop();
  
  // Mark current cycle as used
  void cycleUsed();
  
  // Mark current cycle as performed
  void cyclePerformed();
  
  // Get counter of used cycles
  Counter& getUsedCounter();

private:

  bool used;

  Counter usedCycles;

};

#endif
