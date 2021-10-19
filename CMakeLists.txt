cmake_minimum_required(VERSION 3.12)
project(cartographer_without_ros)
add_executable(cartographer_no_ros src/main.cpp)
  
#---------------------------------------------------------------------------------------------------------------------
# Configure dependencies

# Boost
find_package(Boost REQUIRED COMPONENTS system iostreams)
target_include_directories(cartographer_no_ros PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(cartographer_no_ros PUBLIC ${Boost_LIBRARIES})

# PCL
find_package(PCL REQUIRED COMPONENTS common)
target_include_directories(cartographer_no_ros PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cartographer_no_ros PUBLIC ${PCL_LIBRARIES})

# Cartoghrapher
find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()
target_link_libraries(cartographer_no_ros PUBLIC cartographer)

# STD Filesystem
if(UNIX)
target_link_libraries(cartographer_no_ros LINK_PUBLIC stdc++fs)
endif()