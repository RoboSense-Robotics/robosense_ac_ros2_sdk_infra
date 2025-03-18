cmake_minimum_required(VERSION 3.5)
project(ros2_codec)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

execute_process(
  COMMAND bash -c "cat /proc/cpuinfo | grep 'Rockchip RK3588' | uniq | cut -d ':' -f 2"
  OUTPUT_VARIABLE CPU_MODEL
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(CPU_MODEL STREQUAL " Rockchip RK3588")
  message(STATUS "Detected RK3588 chip")
  set(TARGET_ARCH "RK3588")
  add_definitions(-DRK3588)
  add_definitions(-DH265_DEF)
else()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)$")
    set(TARGET_ARCH "ARM")
    add_definitions(-DARM_ARCH)
    add_definitions(-DH265_DEF)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i686)$")
    set(TARGET_ARCH "X86")
    add_definitions(-DX86_ARCH)
    add_definitions(-DJPEG_DEF)
    include_directories(/usr/include/opencv4/)
else()
    message(STATUS "Unknown target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(TARGET_ARCH "UNKNOWN")
endif()
endif()

#av_codec#
find_library(AVCODEC_LIBRARY avcodec)
find_library(AVFORMAT_LIBRARY avformat)
find_library(SWSCALE_LIBRARY swscale)
find_library(AVUTIL_LIBRARY avutil)
find_library(AVDEVICE_LIBRARY avdevice)
find_library(AVFILTER_LIBRARY avfilter)
find_library(POSTPROC_LIBRARY postproc)
find_library(SWRESAMPLE_LIBRARY swresample)
find_library(LZMA_LIBRARY lzma)
find_library(M_LIBRARY m)
find_library(Z_LIBRARY z)

#ROS2#
find_package(rclcpp QUIET)
if(TARGET_ARCH STREQUAL "X86" OR TARGET_ARCH STREQUAL "ARM")
    find_package(OpenCV REQUIRED)
endif()

if(rclcpp_FOUND)

  add_definitions(-DROS2_FOUND)

  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(robosense_msgs REQUIRED)

else(rclcpp_FOUND)
  message(=============================================================)
  message("-- ROS2 Not Found. ROS2 Support is turned Off.")
  message(=============================================================)
endif(rclcpp_FOUND)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#Ros2#
if(rclcpp_FOUND)

if(TARGET_ARCH STREQUAL "RK3588")
  add_executable(codec_node src/codec_node.cpp)
elseif(TARGET_ARCH STREQUAL "X86" OR TARGET_ARCH STREQUAL "ARM")
  add_executable(codec_node src/codec_node.cpp src/jpegcoder.cpp src/opencvjpegencoder.cpp src/opencvjpegdecoder.cpp src/colorcodec.cpp src/h265coder.cpp)
endif()

  ament_target_dependencies(codec_node rclcpp sensor_msgs robosense_msgs)
  target_link_libraries(codec_node ${AVDEVICE_LIBRARY})
  target_link_libraries(codec_node ${AVFILTER_LIBRARY})
  target_link_libraries(codec_node ${AVCODEC_LIBRARY})
  target_link_libraries(codec_node ${POSTPROC_LIBRARY})
  target_link_libraries(codec_node ${AVFORMAT_LIBRARY})
  target_link_libraries(codec_node ${AVUTIL_LIBRARY})
  target_link_libraries(codec_node ${SWSCALE_LIBRARY})
  target_link_libraries(codec_node ${SWRESAMPLE_LIBRARY})
  target_link_libraries(codec_node ${LZMA_LIBRARY})
  target_link_libraries(codec_node ${M_LIBRARY})
  target_link_libraries(codec_node ${Z_LIBRARY})
if(TARGET_ARCH STREQUAL "RK3588")
  find_library(NUMA_LIBRARY numa)
  target_link_libraries(codec_node ${NUMA_LIBRARY})
  find_library(MPP_LIBRARY rockchip_mpp)
  target_link_libraries(codec_node ${MPP_LIBRARY})
  find_library(RGA_LIBRARY rga)
  target_link_libraries(codec_node ${RGA_LIBRARY})
  find_library(DRM_LIBRARY drm)
  target_link_libraries(codec_node ${DRM_LIBRARY})
  find_library(BZ2_LIBRARY bz2)
  target_link_libraries(codec_node ${BZ2_LIBRARY})
  find_library(BZ2_LIBRARY bz2)
  target_link_libraries(codec_node ${BZ2_LIBRARY})
  find_library(XCB_LIBRARY xcb)
  target_link_libraries(codec_node ${XCB_LIBRARY})
elseif(TARGET_ARCH STREQUAL "X86" OR TARGET_ARCH STREQUAL "ARM")
  target_link_libraries(codec_node ${OpenCV_LIBS})
endif()

  set_target_properties(codec_node PROPERTIES INSTALL_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

  install(TARGETS codec_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
          DESTINATION include/${PROJECT_NAME}
  )

  ament_package()
endif(rclcpp_FOUND)
