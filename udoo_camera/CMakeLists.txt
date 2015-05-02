# udoo_camera/CMakeList.txt
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

project(udoo_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_node src/camera_node.cpp)

target_link_libraries(camera_node 
  ${catkin_LIBRARIES}
)
