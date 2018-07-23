# darknet/CMakeList.txt
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

project(darknet)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  DEPENDS roscpp
)

set(DARKNET_PATH "src/darknet")

add_definitions(-O4 -g)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${DARKNET_PATH}/include
  ${DARKNET_PATH}/src
)

add_library(
  ${PROJECT_NAME}
  ${DARKNET_PATH}/src/activation_layer.c
  ${DARKNET_PATH}/src/im2col.c
  ${DARKNET_PATH}/src/activations.c
  ${DARKNET_PATH}/src/image.c
  ${DARKNET_PATH}/src/avgpool_layer.c
  ${DARKNET_PATH}/src/layer.c
  ${DARKNET_PATH}/src/batchnorm_layer.c
  ${DARKNET_PATH}/src/list.c
  ${DARKNET_PATH}/src/blas.c
  ${DARKNET_PATH}/src/local_layer.c
  ${DARKNET_PATH}/src/box.c
  ${DARKNET_PATH}/src/lstm_layer.c
  ${DARKNET_PATH}/src/col2im.c
  ${DARKNET_PATH}/src/matrix.c
  ${DARKNET_PATH}/src/connected_layer.c
  ${DARKNET_PATH}/src/maxpool_layer.c
  ${DARKNET_PATH}/src/convolutional_layer.c
  ${DARKNET_PATH}/src/network.c
  ${DARKNET_PATH}/src/cost_layer.c
  ${DARKNET_PATH}/src/normalization_layer.c
  ${DARKNET_PATH}/src/crnn_layer.c
  ${DARKNET_PATH}/src/option_list.c
  ${DARKNET_PATH}/src/crop_layer.c
  ${DARKNET_PATH}/src/parser.c
  ${DARKNET_PATH}/src/cuda.c
  ${DARKNET_PATH}/src/region_layer.c
  ${DARKNET_PATH}/src/data.c
  ${DARKNET_PATH}/src/reorg_layer.c
  ${DARKNET_PATH}/src/deconvolutional_layer.c
  ${DARKNET_PATH}/src/rnn_layer.c
  ${DARKNET_PATH}/src/demo.c
  ${DARKNET_PATH}/src/route_layer.c
  ${DARKNET_PATH}/src/detection_layer.c
  ${DARKNET_PATH}/src/shortcut_layer.c
  ${DARKNET_PATH}/src/dropout_layer.c
  ${DARKNET_PATH}/src/softmax_layer.c
  ${DARKNET_PATH}/src/gemm.c
  ${DARKNET_PATH}/src/tree.c
  ${DARKNET_PATH}/src/gru_layer.c
  ${DARKNET_PATH}/src/utils.c
  ${DARKNET_PATH}/src/upsample_layer.c
  ${DARKNET_PATH}/src/logistic_layer.c
  ${DARKNET_PATH}/src/l2norm_layer.c
  ${DARKNET_PATH}/src/yolo_layer.c
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


