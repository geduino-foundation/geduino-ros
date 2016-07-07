# range_layer/CMakeList.txt
#
# Copyright (C) 2016 Alessandro Francescon
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

cmake_minimum_required(VERSION 2.8.3)

project(range_layer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  dynamic_reconfigure
  sensor_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(cfg/RangeLayer.cfg)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(range_layer src/range_layer.cpp)

add_dependencies(range_layer range_layer_gencfg)

target_link_libraries(range_layer
  ${catkin_LIBRARIES}
)
