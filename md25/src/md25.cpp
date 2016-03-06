/*
 md25.cpp
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
 
#include "md25.h"
#include <iostream>
#include <exception>

/* Public methods */

MD25::MD25(std::string port, uint32_t baudrate, uint32_t timeout) {

	// Setup serial
	md25serial.setPort(port);
	md25serial.setBaudrate(baudrate);
	md25serial.setTimeout(std::numeric_limits<uint32_t>::max(), timeout, 0, timeout, 0);

}

uint8_t MD25::init() {

	try {

		// Open serial
		md25serial.open();

		return MD25_RESPONSE_OK;

	} catch (std::exception & ex) {

		// Log
		std::cerr << "Unhandled exception: " << ex.what() << std::endl;

		return MD25_RESPONSE_ERROR;

	}

}

uint8_t MD25::dispose() {

	try {

		// Close serial
		md25serial.close();

		return MD25_RESPONSE_OK;

	} catch (std::exception & ex) {

		// Log
		std::cerr << "Unhandled exception: " << ex.what() << std::endl;

		return MD25_RESPONSE_ERROR;

	}

}

uint8_t MD25::getSpeed1(uint8_t * speed1) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_SPEED_1;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint8(speed1);

	return result;

}

uint8_t MD25::getSpeed2(uint8_t * speed2) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_SPEED_2;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint8(speed2);

	return result;

}

uint8_t MD25::getEncoder1(uint32_t * encoder1) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_ENCODER_1;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint32(encoder1);

	return result;

}

uint8_t MD25::getEncoder2(uint32_t * encoder2) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_ENCODER_2;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint32(encoder2);

	return result;

}

uint8_t MD25::getEncoders(uint32_t * encoder1, uint32_t * encoder2) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_ENCODERS;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = (readUint32(encoder1) == MD25_RESPONSE_OK &&
			readUint32(encoder2) == MD25_RESPONSE_OK) ? MD25_RESPONSE_OK : MD25_RESPONSE_ERROR;

	return result;

}

uint8_t MD25::getVolts(uint8_t * volts) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_VOLTS;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint8(volts);

	return result;

}

uint8_t MD25::getCurrent1(uint8_t * current1) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_CURRENT_1;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint8(current1);

	return result;

}

uint8_t MD25::getCurrent2(uint8_t * current2) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_CURRENT_2;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint8(current2);

	return result;

}

uint8_t MD25::getVersion(uint8_t * version) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_VERSION;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint8(version);

	return result;

}

uint8_t MD25::getAcceleration(uint8_t * acceleration) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_ACCELERATION;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint8(acceleration);

	return result;

}

uint8_t MD25::getMode(uint8_t * mode) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_MODE;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = readUint8(mode);

	return result;

}

uint8_t MD25::getVI(uint8_t * volts, uint8_t * current1, uint8_t * current2) {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = GET_VI;

	// Write command
	write(cmd, 2);

	// Read result
	uint8_t result = (readUint8(volts) == MD25_RESPONSE_OK &&
			readUint8(current1) == MD25_RESPONSE_OK &&
			readUint8(current2) == MD25_RESPONSE_OK) ? MD25_RESPONSE_OK : MD25_RESPONSE_ERROR;

	return result;

}

uint8_t MD25::setSpeed1(uint8_t speed1) {

	// Create command
	uint8_t cmd[3];
	cmd[0] = CMD;
	cmd[1] = SET_SPEED_1;
	cmd[2] = speed1;

	// Write command
	uint8_t result = write(cmd, 3);

	return result;

}

uint8_t MD25::setSpeed2(uint8_t speed2) {

	// Create command
	uint8_t cmd[3];
	cmd[0] = CMD;
	cmd[1] = SET_SPEED_2;
	cmd[2] = speed2;

	// Write command
	uint8_t result = write(cmd, 3);

        return result;

}

uint8_t MD25::setAcceleration(uint8_t acceleration) {

	// Create command
	uint8_t cmd[3];
	cmd[0] = CMD;
	cmd[1] = SET_ACCELERATION;
	cmd[2] = acceleration;

	// Write command
	write(cmd, 3);

}

uint8_t MD25::setMode(uint8_t mode) {

	// Create command
	uint8_t cmd[3];
	cmd[0] = CMD;
	cmd[1] = SET_MODE;
	cmd[2] = mode;

	// Write command
	write(cmd, 3);

}

uint8_t MD25::resetEncoders() {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = RESET_ENCODERS;

	// Write command
	write(cmd, 2);

}

uint8_t MD25::disableRegulator() {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = DISABLE_REGULATOR;

	// Write command
	write(cmd, 2);

}

uint8_t MD25::enabledRegulator() {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = ENABLE_REGULATOR;

	// Write command
	write(cmd, 2);

}

uint8_t MD25::disableTimeout() {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = DISABLE_TIMEOUT;

	// Write command
	write(cmd, 2);

}

uint8_t MD25::enabledTimeout() {

	// Create command
	uint8_t cmd[2];
	cmd[0] = CMD;
	cmd[1] = ENABLE_TIMEOUT;

	// Write command
	write(cmd, 2);

}

/* Private methods */

uint8_t MD25::write(uint8_t * cmd, size_t count) {

	try {

		// Write
		size_t writeCount = md25serial.write(cmd, count);

	        return count == writeCount ? MD25_RESPONSE_OK : MD25_RESPONSE_ERROR;

	} catch (std::exception & ex) {

		// Log
		std::cerr << "Unhandled exception: " << ex.what() << std::endl;

		return MD25_RESPONSE_ERROR;

	}

}

uint8_t MD25::readUint8(uint8_t * value) {

	try {

		// Read one byte
		size_t count = md25serial.read(value, 1);

		return count == 1 ? MD25_RESPONSE_OK : MD25_RESPONSE_ERROR;

	} catch (std::exception & ex) {

		// Log
		std::cerr << "Unhandled exception: " << ex.what() << std::endl;

		return MD25_RESPONSE_ERROR;

	}

}

uint8_t MD25::readUint32(uint32_t * value) {

	uint8_t buffer[4];

	try {

		// Read four bytes
		size_t count = md25serial.read(buffer, 4);

		if (count == 4) {

			// Get value from buffer
			*value = buffer[0];
			*value <<= 8;
			*value += buffer[1];
			*value <<= 8;
			*value += buffer[2];
			*value <<= 8;
			*value += buffer[3];

			return MD25_RESPONSE_OK;

		} else {

			return MD25_RESPONSE_ERROR;

		}

	} catch (std::exception & ex) {

		// Log
		std::cerr << "Unhandled exception: " << ex.what() << std::endl;

		return MD25_RESPONSE_ERROR;

	}

}
