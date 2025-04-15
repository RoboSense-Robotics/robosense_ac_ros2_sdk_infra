cmake_minimum_required(VERSION 3.5)
#========================
# Project
#========================

set(CUR_LIB cuda_nv12_remap_norm)

project(${CUR_LIB} LANGUAGES CXX CUDA)

set(CMAKE_BUILD_TYPE Release)

add_compile_options(-w)
add_compile_options(-std=c++14)
add_definitions(-DDEBUG_LEVEL=0)
add_definitions(-DDEBUGMY)
add_compile_options(-fPIC)
add_compile_options(-Wno-deprecated-declarations)

FIND_PACKAGE(CUDA)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" CACHE PATH "Library output dir.")

link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

cuda_add_library(${CUR_LIB} SHARED
    src/cuda_nv12_remap_norm.cu
)

# 添加安装规则
install(TARGETS ${CUR_LIB}
    DESTINATION lib
)
