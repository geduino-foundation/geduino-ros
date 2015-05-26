# Power.msg
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

# The Geduino power message

# The ac adapter type
int8 TYPE_AC_ADAPTER = 0

# The four cells LiPo type
int8 TYPE_LIPO_4S = 1

# The conversion constant from raw_volt to volt 
float32 RAW_TO_VOLT = 0.0173

# The power type
int8 type

# The raw voltage
int8 raw_voltage
