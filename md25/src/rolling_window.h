/*
 rolling_window.h

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

#ifndef _ROLLING_WINDOW_H
#define _ROLLING_WINDOW_H

#include <stdlib.h>

/*
 Provide class to handle a rolling window of double
 */
class RollingWindow {

   public:

      // Create a rolling window with given size
      RollingWindow(char _size) : size(_size) {
          window = (double *) malloc(_size * sizeof(double));
          reset();
      };

      ~RollingWindow() {
          free(window);
      };

      // Add an value to rolling window
      void add(double value);

      // Get sum of element in the rolling window
      void sum(double * sum);

      // Reset values in rolling window to 0
      void reset();

   private:

      // The rolling window size
      char size;

      // The current rolling window index
      char index;

      // The window
      double * window;

};

#endif
