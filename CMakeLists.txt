# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(MyProject)

# Set the C++ standard (optional)
set(CMAKE_CXX_STANDARD 14)

# Add the executable and include all source files
add_executable(MyProject 
    main.cpp 
    doublependelum.cpp 
    pendelum.cpp
)

# Include directories (adjust the path to Eigen if necessary)
include_directories(${CMAKE_SOURCE_DIR}/eigen)

