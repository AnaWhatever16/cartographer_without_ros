#------------------------------------------------------------------------------------------------------------------------------
# Copyright (c) 2021 Ana Maria Casado Fauli (anacasadofauli@gmail.com)
#------------------------------------------------------------------------------------------------------------------------------
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
# documentation files (the "Software"), to deal in the Software without restriction, including without 
# limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
# Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO 
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#------------------------------------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.12)
project(cartographer_without_ros)

file(GLOB_RECURSE CARTOGRAPHERWITHOUTROS_HDR "include/cartographer_without_ros/*.h")
file(GLOB_RECURSE CARTOGRAPHERWITHOUTROS_SRC "src/cartographer_without_ros/*.cpp")

add_library(cartographer_without_ros STATIC ${CARTOGRAPHERWITHOUTROS_SRC} ${CARTOGRAPHERWITHOUTROS_HDR})
target_include_directories(cartographer_without_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

#---------------------------------------------------------------------------------------------------------------------
# Configure dependencies

# Boost
find_package(Boost REQUIRED COMPONENTS system iostreams)
target_include_directories(cartographer_without_ros PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(cartographer_without_ros PUBLIC ${Boost_LIBRARIES})

# PCL
find_package(PCL REQUIRED COMPONENTS common)
target_include_directories(cartographer_without_ros PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cartographer_without_ros PUBLIC ${PCL_LIBRARIES})

#YAML
find_package(yaml-cpp REQUIRED)
target_include_directories(cartographer_without_ros PUBLIC ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(cartographer_without_ros LINK_PUBLIC ${YAML_CPP_LIBRARIES} yaml-cpp)

# Cartoghrapher
find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()
target_link_libraries(cartographer_without_ros PUBLIC cartographer)

add_executable(fake_trajectory main.cpp)
target_link_libraries(fake_trajectory PUBLIC cartographer_without_ros)
