# md25/CMakeList.txt
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

project(md25)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  diagnostic_msgs
  sensor_msgs
  tf
  serial
  message_generation
)

add_message_files(
    FILES
    Euler.msg
    StampedEuler.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  src
)

add_executable(md25_node src/md25_node.cpp src/odometry.cpp src/rolling_window.cpp src/md25.cpp)
add_executable(md25_console src/md25_console.cpp src/md25.cpp)
add_executable(odom_to_euler_node src/odom_to_euler_node.cpp)

target_link_libraries(md25_node
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_console
  ${catkin_LIBRARIES}
)

target_link_libraries(odom_to_euler_node
  ${catkin_LIBRARIES}
)
