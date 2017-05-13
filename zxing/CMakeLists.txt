# zxing/CMakeList.txt
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

project(zxing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
)

catkin_package(
INCLUDE_DIRS src
LIBRARIES ${PROJECT_NAME}
DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

file(GLOB_RECURSE LIBZXING_FILES
    "src/*.cpp"
    "src/*.h"
    "src/*.cc"
    "src/*.hh"
)

add_library(
  ${PROJECT_NAME}
  ${LIBZXING_FILES}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


