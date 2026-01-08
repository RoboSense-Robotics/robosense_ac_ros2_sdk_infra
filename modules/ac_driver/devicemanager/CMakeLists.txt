cmake_minimum_required(VERSION 3.5)

#========================
# Project
#========================

set(CUR_LIB device)
project(${CUR_LIB})

# Set C++ standard via CMake target properties in srcs.cmake or here
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DDEBUG_LEVEL=0)

include(cmake/srcs.cmake)

# No PARENT_SCOPE needed if we use targets properly


