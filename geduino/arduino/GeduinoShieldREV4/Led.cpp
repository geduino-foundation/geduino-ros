/*
 Led.cpp
 
 Copyright (C) 2017 Alessandro Francescon

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
#include "Led.h"

#define LED_BLINK_SLOW_PERIOD   500
#define LED_BLINK_FAST_PERIOD   125

void Led::ledOn() {

  if (!led_on_status) {

    // Set led to HIGH
    digitalWrite(led_pin, HIGH);

    // Set led status
    led_on_status = true;

  }

}

void Led::ledOff() {

  if (led_on_status) {

    // Set led to LOW
    digitalWrite(led_pin, LOW);

    // Set led status
    led_on_status = false;

  }

}


void Led::ledBlinkSlow() {

  // Led blink
  ledBlink(LED_BLINK_SLOW_PERIOD);

}

void Led::ledBlinkFast() {

  // Led blink
  ledBlink(LED_BLINK_FAST_PERIOD);

}

void Led::ledBlinkSlowFor(const uint32_t & duration) {

  // Led blink for
  ledBlinkFor(LED_BLINK_SLOW_PERIOD, duration);

}

void Led::ledBlinkFastFor(const uint32_t & duration) {

  // Led blink for
  ledBlinkFor(LED_BLINK_FAST_PERIOD, duration);
  
}

void Led::init() {

  // Set led PIN to OUTPUT
  pinMode(led_pin, OUTPUT);

  // Init led status
  led_on_status = false;

  // Set led to LOW
  digitalWrite(led_pin, LOW);

  // Init last blink millis
  last_blink_millis = 0;

}

void Led::ledBlink(const uint32_t & period) {
  
  // Get now
  uint32_t now = millis();

  if (now - last_blink_millis > period) {

    // Toggle led status
    led_on_status = !led_on_status;

    // Invert led PIN status
    digitalWrite(led_pin, led_on_status ? HIGH : LOW);

    // Reset last blink millis
    last_blink_millis = now;

  }

}

void Led::ledBlinkFor(const uint32_t & period, const uint32_t & duration) {

  // Get start time
  const uint32_t start = millis();
  
  // White for a byte
  while (duration == 0 || millis() - start < duration) {

    // Led blink
    ledBlink(period);

    // Just wait half period
    delay(period / 10);
    
  }
  
}
