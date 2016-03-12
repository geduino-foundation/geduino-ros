/*
 PowerBoardREV3.ino
 http://geduino.blogspot.it/

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

#include "Button.h"

#define POWER_BUTTON_PIN 11
#define EXTERNAL_POWER_OFF_BUTTON_PIN 5
#define BATTERY_PIN A0
#define RELE_PIN 7

//#define DEBUG
#define POWER_BUTTON_DELAY 1000
#define EXTERNAL_POWER_OFF_BUTTON_DELAY 2500

Button powerButton(POWER_BUTTON_PIN, POWER_BUTTON_DELAY);
Button externalPowerOffButton(EXTERNAL_POWER_OFF_BUTTON_PIN, EXTERNAL_POWER_OFF_BUTTON_DELAY);

int powerStatus = LOW;

/****************************************************************************************
 * Setup
 */
 
void setup() {
  
#ifdef DEBUG

  // Start serial
  Serial.begin(115200);
  
#endif

  // Set pin direction and initial value
  pinMode(BATTERY_PIN, INPUT);
  pinMode(RELE_PIN, OUTPUT);
  digitalWrite(RELE_PIN, LOW);

#ifdef DEBUG

  // Log
  Serial.println("Setup completed");
  
#endif

}

/****************************************************************************************
 * Loop
 */
 
void loop() {
  
  // Get power button status
  boolean powerButtonStatusChanged;
  int powerButtonStatus = powerButton.getStatus(&powerButtonStatusChanged);
  
#ifdef DEBUG

  if (powerButtonStatusChanged) {

    // Log
    Serial.print("Power button state: ");
    Serial.println(powerButtonStatus);
  
  }
  
#endif

  // Get external power off button status
  boolean externalPowerOffButtonStatusChanged;
  int externalPowerOffButtonStatus = externalPowerOffButton.getStatus(&externalPowerOffButtonStatusChanged);
  
#ifdef DEBUG

  if (externalPowerOffButtonStatusChanged) {

    // Log
    Serial.print("External power off button state: ");
    Serial.println(externalPowerOffButtonStatus);
  
  }
  
#endif

  if (powerButtonStatusChanged && powerButtonStatus == HIGH) {
    
    if (powerStatus == HIGH) {
      
      // Power off
      powerOff();
      
    } else {
      
      // Power on
      powerOn();
      
    }
    
  } else if (externalPowerOffButtonStatusChanged && externalPowerOffButtonStatus == HIGH && powerStatus == HIGH) {
    
    // Power off
    powerOff();
      
  }
  
}

void powerOn() {
  
#ifdef DEBUG

  // Log
  Serial.println("Switching power on...");
 
#endif

  // Set rele pin to high
  digitalWrite(RELE_PIN, HIGH);
 
  // Get external power off button status
  boolean externalPowerOffButtonStatusChanged;
  int externalPowerOffButtonStatus = externalPowerOffButton.getStatus(&externalPowerOffButtonStatusChanged);
  
  while (externalPowerOffButtonStatus == LOW) {
 
#ifdef DEBUG

  // Log
  Serial.println("Still waiting for external power off became LOW...");
 
#endif

    // Get external power off button status
    externalPowerOffButtonStatus = externalPowerOffButton.getStatus(&externalPowerOffButtonStatusChanged);

    // Just wait
    delay(100);
  
  }
  
  // Set power status to HIGH
  powerStatus = HIGH;

#ifdef DEBUG

  // Log
  Serial.println("Done!");
  
#endif

}

void powerOff() {

#ifdef DEBUG

  // Log
  Serial.print("Switching power off... ");
  
#endif

  // Set rele pin to low
  digitalWrite(RELE_PIN, LOW);

  // Set power status to LOW
  powerStatus = LOW;

#ifdef DEBUG

  // Log
  Serial.println("Done!");
  
#endif
}
