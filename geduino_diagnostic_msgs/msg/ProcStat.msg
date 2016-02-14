# ProcStat.msg
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

# The Geduino statistic message

# The number of cycles in main loop in which some action was performed
uint32 used_cycles

# The number of cycles in main loop in which no action was performed
uint32 idle_cycles

# The delay and duration in perform encoders and motion9 calculation
geduino_diagnostic_msgs/Counter encoders_motion9_delay
geduino_diagnostic_msgs/Counter encoders_motion9_duration

# The delay and duration in perform range calculation
geduino_diagnostic_msgs/Counter range_delay
geduino_diagnostic_msgs/Counter range_duration

# The delay and duration in perform diagnostic calculation
geduino_diagnostic_msgs/Counter diagnostic_delay
geduino_diagnostic_msgs/Counter diagnostic_duration


