cmake_minimum_required(VERSION 3.5)
project(robosense_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# detect ros version
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  find_package(catkin QUIET)
  find_package(ament_cmake QUIET)

  if(catkin_FOUND)
    set(ROS_VERSION 1)
  elseif(ament_cmake_FOUND)
    set(ROS_VERSION 2)
  else()
    message(FATAL_ERROR "Can not detect ROS version automatically")
  endif()
endif()

message(STATUS "Building for ROS${ROS_VERSION}")

# build for ROS1
if(ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
  )

  add_message_files(
    FILES
    RsCompressedImage.msg
    RslidarPacket.msg
    RslidarScans.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
  )

  include_directories(${catkin_INCLUDE_DIRS})

# build for ROS2
elseif(ROS_VERSION EQUAL 2)
  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RsCompressedImage.msg"
    "msg/RslidarPacket.msg"
    "msg/RslidarScans.msg"
    DEPENDENCIES builtin_interfaces std_msgs
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()
