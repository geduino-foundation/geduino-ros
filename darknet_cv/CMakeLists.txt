# darknet_cv/CMakeList.txt
#
# Copyright (C) 2018 Alessandro Francescon
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

project(darknet_cv)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    message_generation
    image_transport
    std_msgs
    sensor_msgs
    darknet
)

add_message_files(
    FILES
    DetectionClass.msg
    Detection.msg
    DetectionArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  DEPENDS roscpp
  CATKIN_DEPENDS message_runtime
)

message(${catkin_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(yolo_detector_node src/yolo_detector_node.cpp src/yolo_detector.cpp)

target_link_libraries(yolo_detector_node
        ${catkin_LIBRARIES}
)
