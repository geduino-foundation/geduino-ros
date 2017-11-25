/*
 Led.h
 
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

#ifndef _LED_H_
#define _LED_H_

class Led {

public:

    Led(const uint8_t _led_pin) : led_pin(_led_pin) {
    }

    void init();

    void ledOn();

    void ledOff();

    void ledBlinkSlow();

    void ledBlinkFast();

    void ledBlinkSlowFor(const uint32_t & duration);

    void ledBlinkFastFor(const uint32_t & duration);

  private:

    const uint8_t led_pin;

    uint32_t last_blink_millis;

    bool led_on_status;

    void ledBlink(const uint32_t & period);

    void ledBlinkFor(const uint32_t & period, const uint32_t & duration);
  
};

#endif


