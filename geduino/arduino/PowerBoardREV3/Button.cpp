/*
 Button.cpp
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
 
#include <Arduino.h>
#include "Button.h"

/* Public methods */

int Button::getStatus(boolean * changed) {
  
  // Read current button status
  int currentButtonStatus = digitalRead(buttonPin);
 
  // Get now
  long now = millis();
  
  if (currentButtonStatus != lastButtonStatus) {
    
    // Reset last button change time
    lastButtonChangeTime = now;
    
    // Reset last button status
    lastButtonStatus = currentButtonStatus;
    
  }
  
  if ((now - lastButtonChangeTime) > buttonDelay) {
    
    // Set changed
    *changed = lastStableButtonStatus != currentButtonStatus;
    
    // Set last stable button status
    lastStableButtonStatus = currentButtonStatus;
    
  } else {
    
    // Set changed to false
    *changed = false;
    
  }
  
  return lastStableButtonStatus;
    
}
