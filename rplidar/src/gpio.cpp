/*
 gpio.cpp

 Copyright (C) 2015 Alessandro Francescon
 
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

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include "gpio.h"

#define EXPORT_FILE_PATH "/sys/class/gpio/export"
#define UNEXPORT_FILE_PATH "/sys/class/gpio/unexport"
#define GPIO_DIRECTION_FILE_PATH "/sys/class/gpio/gpio" + number + "/direction"
#define GPIO_VALUE_FILE_PATH "/sys/class/gpio/gpio" + number + "/value"

int GPIO::exportGPIO() {

	// Write GPIO number to export file
	return writeToFile(EXPORT_FILE_PATH, number);

}

int GPIO::unexportGPIO() {

	// Write GPIO number to unexport file
	return writeToFile(UNEXPORT_FILE_PATH, number);

}

int GPIO::setDirection(std::string direction) {

	// Write direction to GPIO direction file
	return writeToFile(GPIO_DIRECTION_FILE_PATH, direction);

}

int GPIO::setValue(std::string value) {

	// Write value to GPIO value file
	return writeToFile(GPIO_VALUE_FILE_PATH, value);

}

int GPIO::getValue(std::string & value) {

	std::string gpioValueFilePath = GPIO_VALUE_FILE_PATH;

	// Open value file stream
	std::ifstream valueFileStream(gpioValueFilePath.c_str());

	if (valueFileStream < 0) {

		// Err
		fprintf(stderr, "could not open file %s", gpioValueFilePath.c_str());

		return -1;

	}

	// Read GPIO value
	valueFileStream >> value;

	if (value != VALUE_LOW) {

		// Force value to high if value is not low
		value = VALUE_HIGH;

	}

	// Close value file stream
	valueFileStream.close();

	return 0;

}

int GPIO::writeToFile(std::string filePath, std::string string) {

	// Open stream to file
	std::ofstream fileStream(filePath.c_str());

	if (fileStream < 0) {

		// Err
		fprintf(stderr, "could not open file %s", filePath.c_str());

		return -1;

	}

	// Write string to file stream
	fileStream << string;

	// Close file stream
	fileStream.close();

	return 0;

}
