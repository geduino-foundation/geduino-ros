# rplidar/CMakeList.txt
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


cmake_minimum_required(VERSION 2.8.3)

project(rplidar)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  diagnostic_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)

catkin_package()

add_executable(rplidar_node src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
add_executable(rplidar_off src/rplidar_off.c)

target_link_libraries(rplidar_node
	${catkin_LIBRARIES}
)

target_link_libraries(rplidar_off
	${catkin_LIBRARIES}
)
