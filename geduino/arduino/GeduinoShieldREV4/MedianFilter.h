/*
 MedianFilter.h

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
 
#ifndef _MEDIAN_FILTER_H_
#define _MEDIAN_FILTER_H_

// The median filter window size
#define WINDOW_SIZE 10

// The outliers to be excluded by the filter (use odd number)
#define OUTLIERS 6

class MedianFilter {

public:

  // Create new median filter
  MedianFilter();
  
  // Filter given raw data and set result on filtered. The filter
  // will calculate filtered value based on median of values in window
  // where upper and lower values are excluded as outliers.
  void filter(float raw, float * filtered);

private:
  
  // The window
  float window[WINDOW_SIZE];
  
  // The current index in window
  int index;
  
  // Get the median value in window
  void median(float * median);

};

#endif
