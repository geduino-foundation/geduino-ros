/*
 md25.h
 Implementation based on technical specification available at:
 http://www.robot-electronics.co.uk/htm/md25ser.htm#speed
 
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

#ifndef _MD25_H_
#define _MD25_H_

#include <serial/serial.h>

#define MD25_RESPONSE_OK 0x00
#define MD25_RESPONSE_ERROR 0x01

#define CMD 0x00
#define GET_SPEED_1 0x21
#define GET_SPEED_2 0x22
#define GET_ENCODER_1 0x23
#define GET_ENCODER_2 0x24
#define GET_ENCODERS 0x25
#define GET_VOLTS 0x26
#define GET_CURRENT_1 0x27
#define GET_CURRENT_2 0x28
#define GET_VERSION 0x29
#define GET_ACCELERATION 0x2A
#define GET_MODE 0x2B
#define GET_VI 0x2C
#define SET_SPEED_1 0x31
#define SET_SPEED_2 0x32
#define SET_ACCELERATION 0x33
#define SET_MODE 0x34
#define RESET_ENCODERS 0x35
#define DISABLE_REGULATOR 0x36
#define ENABLE_REGULATOR 0x37
#define DISABLE_TIMEOUT 0x38
#define ENABLE_TIMEOUT 0x39

class MD25 {

public:

	// Create MD25 on given port, with given baudrate and timeot (in millis)
	MD25(std::string port, uint32_t baudrate, uint32_t timeout);

	// Init MD25 board communication.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t init();

	// Dispose MD25 board communication.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t dispose();

	// Get the current requested speed of motor 1. Depending on what mode you are in,
	// this register can affect the speed of one motor or both motors.
	// If you are in mode 0 or 1 it will set the speed and direction of motor 1.
	// The larger the number written to this register, the more power is applied to the motor.
	// A mode of 2 or 3 will control the speed and direction of both motors (subject to effect
	// of turn register).
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getSpeed1(uint8_t * speed1);

	// Get the current requested speed of motor 2. When in mode 0 or 1 this operates
	// the speed and direction of motor 2. When in mode 2 or 3 Speed2 becomes a Turn value,
	// and is combined with Speed1 to steer the device (see below).
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getSpeed2(uint8_t * speed2);

	// Get motor 1 encoder count.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getEncoder1(uint32_t * encoder1);

	// Get motor 2 encoder count.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getEncoder2(uint32_t * encoder2);

	// Get motor 1 and 2 encoders count.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getEncoders(uint32_t * encoder1, uint32_t * encoder2);

	// Get a reading of the voltage of the connected battery is available. It returns as 10
	// times the voltage (121 for 12.1v).
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getVolts(uint8_t * volts);

	// Get reading of the average current through the motor1 is available. It reads approx ten
	// times the number of Amps (25 at 2.5A).
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getCurrent1(uint8_t * current1);

	// Get reading of the average current through the motor2 is available. It reads approx ten
	// times the number of Amps (25 at 2.5A).
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getCurrent2(uint8_t * current2);

	// Responds with the revision number of the software in the modules PIC16F873 controller,
	// currently 4 at the time of writing.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getVersion(uint8_t * version);

	// Get current acceleration rate.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getAcceleration(uint8_t * acceleration);

	// Get the currently selected mode.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getMode(uint8_t * mode);

	// Get same result as getVolts(), getCurrent1(), getCurrent2() in the same method.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t getVI(uint8_t * volts, uint8_t * current1, uint8_t * current2);

	// Set speed of motor 1
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t setSpeed1(uint8_t speed1);

	// Set speed of motor 2
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t setSpeed2(uint8_t speed2);

	// Set new acceleration. Accept a value between 1 (slow acceleration rate) to 10 (fast
	// acceleration rate).
	// Default value is 5. See MD25 documentation for more details
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t setAcceleration(uint8_t acceleration);

	// Set the way the speed/turn values are used. The options being:
	// 0: the speed values are literal speeds in the range of 0 (full reverse), 128 (stop) and
	//	255 (full forward);
	// 1: the same as 0 except that the speed values are interpreted as signed values. The range
	// 	being -128 (full reverse), 0 (stop) and 127 (full forward);
	// 2: in this mode speed1 control both motors speed, and speed2 becomes the turn value.
	// 	Data is in the range of 0 (full reverse), 128 (stop) and 255 (full forward);
	// 3: the same as 2, except that the speed values are interpreted as signed values. The range
	// 	being -128 (full reverse), 0 (stop) and 127 (full forward).
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t setMode(uint8_t mode);

	// Reset to zero both of the encoder counts
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t resetEncoders();

	// Disable the power output regulator (not changed by encoder feedback)
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t disableRegulator();

	// Enable the power output regulator (changed by encoder feedback)
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
 	uint8_t enabledRegulator();

	// Disable timeout (MD25 will continuously output with no regular commands)
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t disableTimeout();

	// Enable timeout (MD25 output will stop after 2 seconds without communication).
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t enabledTimeout();

private:

	serial::Serial md25serial;

	// Write given command.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t write(uint8_t * cmd, size_t count);

	// Read a byte.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t readUint8(uint8_t * value);

	// Read 4 bytes.
	// Rerturns MD25_RESPONSE_OK if operaation success, MD25_RESPONSE_ERROR if error occurs.
	uint8_t readUint32(uint32_t * value);

};

#endif
