cmake_minimum_required(VERSION 3.5)

#========================
# Project
#========================

set(CUR_LIB device)
project(${CUR_LIB})

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++14)
add_compile_options(-W)
add_compile_options(-std=c++14)
add_definitions(-DDEBUG_LEVEL=0)

include(cmake/srcs.cmake)

set(${CUR_LIB} PARENT_SCOPE)


