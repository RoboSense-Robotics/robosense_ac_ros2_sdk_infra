cmake_minimum_required(VERSION 3.5)
project(ac_driver)

# Set CMake policies for newer versions
if(POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW) # INTERPROCEDURAL_OPTIMIZATION
endif()
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW) # find_package() uses <PackageName>_ROOT variables
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW) # find_package() uses <PackageName>_ROOT variables
endif()

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(
    -O3
    -ffast-math
    -flto
    -fexceptions
    -pthread
)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

set(PLATFORM_TARGET "AUTO" CACHE STRING "Target platform (AUTO, JETSON_ORIN, RK3588, ARM, X86)")

if(PLATFORM_TARGET STREQUAL "AUTO")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)$")
    if(EXISTS "/proc/device-tree/model")
      file(READ "/proc/device-tree/model" COMPATIBLE_CONTENT)
    else()
      set(COMPATIBLE_CONTENT "")
    endif()
    execute_process(
      COMMAND bash -c "cat /proc/cpuinfo | grep 'Rockchip RK3588' | uniq | cut -d ':' -f 2"
      OUTPUT_VARIABLE CPU_MODEL
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(COMPATIBLE_CONTENT MATCHES "Jetson")
      message(STATUS "Detected Orin platform")
      set(TARGET_ARCH "JETSON_ORIN")
    elseif(CPU_MODEL STREQUAL " Rockchip RK3588")
      message(STATUS "Detected RK3588 chip")
      set(TARGET_ARCH "RK3588")
    else()
      message(STATUS "cur model: ${COMPATIBLE_CONTENT}")
      set(TARGET_ARCH "ARM")
    endif()
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i686)$")
    set(TARGET_ARCH "X86")
  else()
    message(STATUS "Unknown target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(TARGET_ARCH "UNKNOWN")
  endif()
else()
  set(TARGET_ARCH ${PLATFORM_TARGET})
  message(STATUS "Manually set target architecture: ${TARGET_ARCH}")
endif()

if(TARGET_ARCH STREQUAL "JETSON_ORIN")
    add_definitions(-DJETSON_ORIN)
elseif(TARGET_ARCH STREQUAL "RK3588")
    add_definitions(-DRK3588)
    add_definitions(-DSUPPORT_H265)
elseif(TARGET_ARCH STREQUAL "ARM")
    add_definitions(-DARM_ARCH)
elseif(TARGET_ARCH STREQUAL "X86")
    add_definitions(-DX86_ARCH)
endif()

#av_codec#
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(SWSCALE REQUIRED libswscale)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(AVDEVICE REQUIRED libavdevice)
pkg_check_modules(AVFILTER REQUIRED libavfilter)
pkg_check_modules(POSTPROC REQUIRED libpostproc)
pkg_check_modules(SWRESAMPLE REQUIRED libswresample)

find_library(LZMA_LIBRARY lzma)
find_library(M_LIBRARY m)
find_library(Z_LIBRARY z)

#ROS2#
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robosense_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_definitions(-DROS2_FOUND)

# PCL # 
find_package(PCL REQUIRED COMPONENTS common io)

# OpenCV 
find_package(OpenCV REQUIRED)

#Eigen 
find_package(Eigen3 REQUIRED)

set(THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/third_party)
#========================
# libusb 
#========================
include_directories(${THIRD_PARTY_DIR}/libusb) 
include_directories(${THIRD_PARTY_DIR}/libusb/libusb) 
add_subdirectory(third_party/libusb)

#
# libuvc 
#
include_directories(${THIRD_PARTY_DIR}/libuvc) 
include_directories(${THIRD_PARTY_DIR}/libuvc/include) 
add_subdirectory(third_party/libuvc)

include_directories(${THIRD_PARTY_DIR}/rga) 

add_definitions(-DENABLE_USB)
add_definitions(-DDISABLE_PCAP_PARSE)

#
# devicemanager 
# 
add_subdirectory(devicemanager)

#Ros2#
find_package(rclcpp_components REQUIRED)

if(TARGET_ARCH STREQUAL "RK3588")
  add_library(ms_node_component SHARED src/metaS_node.cpp)
else()
  add_library(ms_node_component SHARED src/metaS_node.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/src/jpegcoder.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/src/opencvjpegencoder.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/src/opencvjpegdecoder.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/src/colorcodec.cpp)
endif()

target_include_directories(ms_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/devicemanager/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(ms_node_component PUBLIC
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  robosense_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  diagnostic_updater
)

# Standard target linking
target_link_libraries(ms_node_component PUBLIC
  ${AVDEVICE_LIBRARIES}
  ${AVFILTER_LIBRARIES}
  ${AVCODEC_LIBRARIES}
  ${POSTPROC_LIBRARIES}
  ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES}
  ${SWSCALE_LIBRARIES}
  ${SWRESAMPLE_LIBRARIES}
  ${LZMA_LIBRARY}
  ${M_LIBRARY}
  ${Z_LIBRARY}
  PCL::common
  PCL::io
  ${OpenCV_LIBRARIES}
  device
)

# ament_export_targets(ms_node_component HAS_LIBRARY_TARGET) REMOVED
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  robosense_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  diagnostic_updater
  PCL
)

if(TARGET_ARCH STREQUAL "RK3588")
  find_library(NUMA_LIBRARY numa)
  find_library(MPP_LIBRARY rockchip_mpp)
  find_library(RGA_LIBRARY rga)
  find_library(DRM_LIBRARY drm)
  find_library(BZ2_LIBRARY bz2)
  find_library(XCB_LIBRARY xcb)
  find_library(ASOUND_LIBRARY asound REQUIRED)
  find_library(SNDIO_LIBRARY sndio REQUIRED)
  find_library(X11_LIBRARY NAMES X11)
  find_library(XV_LIBRARY NAMES Xv)
  find_library(XEXT_LIBRARY NAMES Xext)
  
  target_link_libraries(ms_node_component
    ${NUMA_LIBRARY}
    ${MPP_LIBRARY}
    ${RGA_LIBRARY}
    ${DRM_LIBRARY}
    ${BZ2_LIBRARY}
    ${XCB_LIBRARY}
    ${ASOUND_LIBRARY}
    ${SNDIO_LIBRARY}
    ${X11_LIBRARY}
    ${XEXT_LIBRARY}
    ${XV_LIBRARY}
  )
elseif(TARGET_ARCH STREQUAL "JETSON_ORIN")
  add_subdirectory(cuda_nv12_remap_norm)
  target_include_directories(ms_node_component PRIVATE /usr/local/cuda/include src)
  
  find_library(CUDA_LIBRARY cuda HINTS /usr/lib/aarch64-linux-gnu/tegra REQUIRED)
  find_library(CUDART_LIBRARY cudart HINTS /usr/local/cuda/lib64 REQUIRED)
  
  target_link_libraries(ms_node_component 
    ${CUDA_LIBRARY}
    ${CUDART_LIBRARY}
    cuda_nv12_remap_norm
  )
endif()

rclcpp_components_register_node(ms_node_component
  PLUGIN "robosense::ac::MSPublisher"
  EXECUTABLE ms_node
)

install(TARGETS ms_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ms_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_ms_node_launch.py)
endif()

ament_package()
