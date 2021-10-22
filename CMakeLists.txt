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
