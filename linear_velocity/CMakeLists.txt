cmake_minimum_required(VERSION 3.4)
project(linear_velocity)

# Specify the path to the Franka library
set(CMAKE_PREFIX_PATH "/home/crta/libfranka/lib/cmake/Franka" ${CMAKE_PREFIX_PATH})

# Find the required packages
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the source files
set(PROJECT_SOURCES
    linear_velocity.cpp
    tcp_mg.cpp
)

# Create an executable from the source files
add_executable(linear_velocity ${PROJECT_SOURCES})

# Specify the include directories
target_include_directories(linear_velocity PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

# Link the required libraries
target_link_libraries(linear_velocity PRIVATE Franka::Franka)