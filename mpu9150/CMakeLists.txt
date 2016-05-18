# mpu9150/CMakeList.txt
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
project(mpu9150)

add_definitions(-DEMPL_TARGET_LINUX)
add_definitions(-DMPU9150)
add_definitions(-DAK8975_SECONDARY)

set(EMPL_PATH "./src/eMPL/")
set(GLUE_PATH "./src/glue/")

FILE(GLOB EMPL_SRC 
  "${EMPL_PATH}/*.c"
)
FILE(GLOB GLUE_SRC 
  "${GLUE_PATH}/*.c"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  diagnostic_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EMPL_PATH}
  ${GLUE_PATH}
  src
)

add_executable(mpu9150_node src/mpu9150_node.cpp src/mpu9150.cpp src/utility.c ${EMPL_SRC} ${GLUE_SRC})
add_executable(imu_quaternion_to_euler_node src/imu_quaternion_to_euler_node.cpp)

target_link_libraries(mpu9150_node
  ${catkin_LIBRARIES}
)
target_link_libraries(imu_quaternion_to_euler_node
  ${catkin_LIBRARIES}
)
