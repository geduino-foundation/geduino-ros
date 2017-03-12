/*
 MedianFilter.cpp

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

#include <Arduino.h>
#include "MedianFilter.h"

/* Public methods */

MedianFilter::MedianFilter() {
    
  index = 0;
  
  for (int index = 0; index < WINDOW_SIZE; index++) {

    // Fill window with zeroes
    window[index] = 0;
  
  }
  
}

void MedianFilter::filter(float raw, float * filtered) {
  
  // Put raw value in window
  window[index++] = raw;
  
  if (index > WINDOW_SIZE) {
    
    // Reset index
    index = 0;
    
  }
  
  // Calculate median filter
  median(filtered);
  
}

/* Private methods */

void MedianFilter::median(float * median) {
  
  float sortedWindow[WINDOW_SIZE];
  
  // Copy window to sorted window
  memcpy(sortedWindow, window, sizeof(window));
  
  int swaps;
  
  do {
    
    swaps = 0;
   
    for (int index = 0; index < (WINDOW_SIZE - 1); index++) {
      
      if (sortedWindow[index] > sortedWindow[index + 1]) {
       
        // Swap
        float swapped = sortedWindow[index];
        sortedWindow[index] = sortedWindow[index + 1];
        sortedWindow[index + 1] = swapped;
        
        swaps++;
        
      }
    
    }
    
  } while (swaps > 0);
      
  float sum = 0;

  for (int index = OUTLIERS / 2; index < (WINDOW_SIZE - OUTLIERS / 2); index++) {
  
    // Add to sum
    sum += window[index];
    
  }
  
  // Get median
  * median = sum / (WINDOW_SIZE - OUTLIERS);
  
}
