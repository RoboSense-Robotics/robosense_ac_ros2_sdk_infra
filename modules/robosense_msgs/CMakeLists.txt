cmake_minimum_required(VERSION 3.5)
project(robosense_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RsHeader.msg"
  "msg/RsPointField.msg"
  "msg/RsPointCloud.msg"
  "msg/RsCompressedImage.msg"
  "msg/RslidarPacket.msg"
  "msg/RslidarScans.msg"
  "msg/RsImage.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
