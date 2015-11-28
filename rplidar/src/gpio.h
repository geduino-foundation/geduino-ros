/*
 gpio.h

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


#ifndef GPIO_H
#define GPIO_H

#include <string>

#define DIRECTION_IN "in"
#define DIRECTION_OUT "out"

#define VALUE_LOW "0"
#define VALUE_HIGH "1"

/*
 A class to handle GPIOs
 */
class GPIO {

	public:

		// Create a GPIO object to contro GPIO with given number
		GPIO(std::string _number) : number(_number) {};

		// Export GPIO
		int exportGPIO();

		// Unexport GPIO
		int unexportGPIO();

		// Set direction
		int setDirection(std::string direction);

		// Set value of GPIO when direction is set to out
		int setValue(std::string value);

		// Get value of GPIO when direction is set to in
		int getValue(std::string & value);

		// Get GPIO number
		std::string getNumber() {
			return number;
		};

	private:

		// Write given string into given file
		int writeToFile(std::string filePath, std::string string);

		// The gpio number
		std::string number;

};

#endif
