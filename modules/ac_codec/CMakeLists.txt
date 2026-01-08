cmake_minimum_required(VERSION 3.5)
project(ac_codec)

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

set(PLATFORM_TARGET "AUTO" CACHE STRING "Target platform (AUTO, RK3588, ARM, X86)")

if(PLATFORM_TARGET STREQUAL "AUTO")
  execute_process(
    COMMAND bash -c "cat /proc/cpuinfo | grep 'Rockchip RK3588' | uniq | cut -d ':' -f 2"
    OUTPUT_VARIABLE CPU_MODEL
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(CPU_MODEL STREQUAL " Rockchip RK3588")
    message(STATUS "Detected RK3588 chip")
    set(TARGET_ARCH "RK3588")
  else()
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)$")
      set(TARGET_ARCH "ARM")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i686)$")
      set(TARGET_ARCH "X86")
    else()
      message(STATUS "Unknown target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
      set(TARGET_ARCH "UNKNOWN")
    endif()
  endif()
else()
  set(TARGET_ARCH ${PLATFORM_TARGET})
  message(STATUS "Manually set target architecture: ${TARGET_ARCH}")
endif()

if(TARGET_ARCH STREQUAL "RK3588")
  add_definitions(-DRK3588)
  add_definitions(-DH265_DEF)
elseif(TARGET_ARCH STREQUAL "ARM")
  add_definitions(-DARM_ARCH)
  add_definitions(-DH265_DEF)
elseif(TARGET_ARCH STREQUAL "X86")
  add_definitions(-DX86_ARCH)
  add_definitions(-DJPEG_DEF)
  include_directories(/usr/include/opencv4/)
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

include_directories(
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${AVDEVICE_INCLUDE_DIRS}
  ${AVFILTER_INCLUDE_DIRS}
  ${POSTPROC_INCLUDE_DIRS}
  ${SWRESAMPLE_INCLUDE_DIRS}
)

#ROS2#
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robosense_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-DROS2_FOUND)

if(TARGET_ARCH STREQUAL "RK3588")
  add_library(codec_node_component SHARED src/codec_node.cpp)
elseif(TARGET_ARCH STREQUAL "X86" OR TARGET_ARCH STREQUAL "ARM")
  add_library(codec_node_component SHARED src/codec_node.cpp src/jpegcoder.cpp src/opencvjpegencoder.cpp src/opencvjpegdecoder.cpp src/colorcodec.cpp src/h265coder.cpp)
endif()

target_include_directories(codec_node_component PRIVATE include)

ament_target_dependencies(codec_node_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  robosense_msgs
  std_msgs
  cv_bridge
)


# Standard target linking

target_link_libraries(codec_node_component
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
  ${OpenCV_LIBS}
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
  
  target_link_libraries(codec_node_component
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
endif()

rclcpp_components_register_node(codec_node_component
  PLUGIN "robosense::ac::CodecPublisher"
  EXECUTABLE codec_node
)

install(TARGETS codec_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS codec_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

ament_package()
