cmake_minimum_required(VERSION 3.5)
project(ac_driver)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
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

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)$")
  file(READ "/proc/device-tree/model" COMPATIBLE_CONTENT)
  execute_process(
    COMMAND bash -c "cat /proc/cpuinfo | grep 'Rockchip RK3588' | uniq | cut -d ':' -f 2"
    OUTPUT_VARIABLE CPU_MODEL
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(COMPATIBLE_CONTENT MATCHES "Jetson")
    message(STATUS "Detected Orin platform")
    set(TARGET_ARCH "JETSON_ORIN")
    add_definitions(-DJETSON_ORIN)
  elseif(CPU_MODEL STREQUAL " Rockchip RK3588")
    message(STATUS "Detected RK3588 chip")
    set(TARGET_ARCH "RK3588")
    add_definitions(-DRK3588)
    add_definitions(-DSUPPORT_H265)
  else()
  message(STATUS "cur model: ${COMPATIBLE_CONTENT}")
    set(TARGET_ARCH "ARM")
    add_definitions(-DARM_ARCH)
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i686)$")
  set(TARGET_ARCH "X86")
  add_definitions(-DX86_ARCH)
  include_directories(/usr/include/opencv4/)
else()
  message(STATUS "Unknown target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  set(TARGET_ARCH "UNKNOWN")
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

if(rclcpp_FOUND)

  add_definitions(-DROS2_FOUND)

  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(robosense_msgs REQUIRED)


  # PCL # 
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  # OpenCV 
  find_package       (OpenCV REQUIRED)
  include_directories(${Opencv_INCLUDE_DIRS})
  link_directories   (${Opencv_LIBRARY_DIRS}) 
  set(OPENCV_LIBS  opencv_core opencv_imgcodecs opencv_imgproc) 

  #Eigen 
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN_INCLUDE_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})

else(rclcpp_FOUND)
  message(=============================================================)
  message("-- ROS2 Not Found. ROS2 Support is turned Off.")
  message(=============================================================)
endif(rclcpp_FOUND)

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
include_directories(${PROJECT_SOURCE_DIR}/devicemanager) 
include_directories(${PROJECT_SOURCE_DIR}/devicemanager/include/)
add_subdirectory(devicemanager)

#Ros2#
if(rclcpp_FOUND)

if(TARGET_ARCH STREQUAL "RK3588")
  add_executable(ms_node src/metaS_node.cpp)
else()
  add_executable(ms_node src/metaS_node.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/src/jpegcoder.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/src/opencvjpegencoder.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/src/opencvjpegdecoder.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/src/colorcodec.cpp)
endif()
  target_link_libraries(ms_node ${SS_LIBRARY})
  ament_target_dependencies(ms_node rclcpp sensor_msgs robosense_msgs)
  target_link_libraries(ms_node ${AVDEVICE_LIBRARY})
  target_link_libraries(ms_node ${AVFILTER_LIBRARY})
  target_link_libraries(ms_node ${AVCODEC_LIBRARY})
  target_link_libraries(ms_node ${POSTPROC_LIBRARY})
  target_link_libraries(ms_node ${AVFORMAT_LIBRARY})
  target_link_libraries(ms_node ${AVUTIL_LIBRARY})
  target_link_libraries(ms_node ${SWSCALE_LIBRARY})
  target_link_libraries(ms_node ${SWRESAMPLE_LIBRARY})
  target_link_libraries(ms_node ${LZMA_LIBRARY})
  target_link_libraries(ms_node ${M_LIBRARY})
  target_link_libraries(ms_node ${Z_LIBRARY})
  target_link_libraries(ms_node ${PCL_LIBRARIES})
  target_link_libraries(ms_node device)
if(TARGET_ARCH STREQUAL "RK3588")
  find_library(NUMA_LIBRARY numa)
  target_link_libraries(ms_node ${NUMA_LIBRARY})
  find_library(MPP_LIBRARY rockchip_mpp)
  target_link_libraries(ms_node ${MPP_LIBRARY})
  find_library(RGA_LIBRARY rga)
  target_link_libraries(ms_node ${RGA_LIBRARY})
  find_library(DRM_LIBRARY drm)
  target_link_libraries(ms_node ${DRM_LIBRARY})
  find_library(BZ2_LIBRARY bz2)
  target_link_libraries(ms_node ${BZ2_LIBRARY})
  find_library(XCB_LIBRARY xcb)
  target_link_libraries(ms_node ${XCB_LIBRARY})

  find_library(ASOUND_LIBRARY asound REQUIRED)
  find_library(SNDIO_LIBRARY sndio REQUIRED)
  find_library(X11_LIBRARY NAMES X11)
  find_library(XV_LIBRARY NAMES Xv)
  find_library(XEXT_LIBRARY NAMES Xext)
  target_link_libraries(ms_node
    ${ASOUND_LIBRARY}
    ${SNDIO_LIBRARY}
    ${X11_LIBRARY}
    ${XEXT_LIBRARY}
    ${XV_LIBRARY}
  )
elseif(TARGET_ARCH STREQUAL "JETSON_ORIN")
  # 添加 cuda_nv12_remap_norm 子模块
  add_subdirectory(cuda_nv12_remap_norm)

  include_directories(/usr/local/cuda/include
    ./src) # 添加 CUDA 头文件路径
  target_include_directories(ms_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/include)
  find_library(CUDA_LIBRARY cuda HINTS /usr/lib/aarch64-linux-gnu/tegra REQUIRED)
  find_library(CUDART_LIBRARY cudart HINTS /usr/local/cuda/lib64 REQUIRED)
  if(NOT CUDA_LIBRARY OR NOT CUDART_LIBRARY)
    message(FATAL_ERROR "Failed to find required libraries for Orin platform: CUDA_LIBRARY or CUDART_LIBRARY")
  endif()
  message(STATUS "Found CUDA_LIBRARY: ${CUDA_LIBRARY}")
  message(STATUS "Found CUDART_LIBRARY: ${CUDART_LIBRARY}")
  target_link_directories(ms_node PRIVATE ./lib)
  target_link_libraries(ms_node 
    ${CUDA_LIBRARY}
    ${CUDART_LIBRARY}
    ${OpenCV_LIBS}
    cuda_nv12_remap_norm
  )
elseif(TARGET_ARCH STREQUAL "X86" OR TARGET_ARCH STREQUAL "ARM")
    target_include_directories(ms_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../ac_codec/include)
    target_link_libraries(ms_node ${OpenCV_LIBS})
endif()

  install(TARGETS ms_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch rviz
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
endif(rclcpp_FOUND)
