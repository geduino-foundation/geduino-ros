/*
 Button.h
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
 
#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <ctype.h>
#include <Arduino.h>

class Button {

public:

  // Create new button
  Button(int _buttonPin, long _buttonDelay) : buttonPin(_buttonPin), buttonDelay(_buttonDelay), lastStableButtonStatus(LOW), lastButtonStatus(LOW), lastButtonChangeTime(0) {
    
    // Set button pin direction
    pinMode(buttonPin, INPUT);
    
  };
  
  // Get button status
  int getStatus(boolean * changed);

private:

  int buttonPin;
  long buttonDelay;
  int lastStableButtonStatus;
  int lastButtonStatus;
  long lastButtonChangeTime;
  
};

#endif
