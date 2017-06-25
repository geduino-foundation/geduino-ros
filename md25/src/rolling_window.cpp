/*
 rolling_window.cpp

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

#include <rolling_window.h>

#include <string.h>

void RollingWindow::add(double value) {

    // Set value on window
    window[index] = value;

    // Increase index
    if (++index >= size) {

       // Reset index
       index = 0;

    }

}

void RollingWindow::sum(double * sum) {

    * sum = 0;

    for (char sumIndex = 0; sumIndex < size; sumIndex++) {
        * sum += window[sumIndex];
    }

}

void RollingWindow::reset() {

    // Fill window with zeroes
    memset(window, 0, sizeof(window));

    // Reset index
    index = 0;

}
