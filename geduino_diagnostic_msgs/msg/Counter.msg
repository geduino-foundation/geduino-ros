# Counter.msg
#
# Copyright (C) 2015 Alessandro Francescon
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# The Geduino counter message. This message is used to send statistic value about
# some unit. It contains a sum and a counter: the sum is the sum of all measurements;
# the counter is a counter of measurements.
# Using this format is easy to calculate average value by sum / counter. 

# The sum of measuments
uint32 sum

# The counter of measurements
uint32 counter

