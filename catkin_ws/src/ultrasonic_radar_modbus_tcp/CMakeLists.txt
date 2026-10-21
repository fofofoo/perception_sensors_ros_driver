cmake_minimum_required(VERSION 3.0.2)
project(ultrasonic_radar_modbus_tcp)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

# 节点内有后台重探线程（std::thread），需要显式链接 pthread。
find_package(Threads REQUIRED)

add_message_files(
  FILES
  ultrasonic_radar_data.msg
  ultrasonic_radar_datas.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ultrasonic_radar_modbus_tcp_node
  src/main.cpp
  src/ultrasonic_radar_node.cpp
  src/modbus_tcp_client.cpp
)

# 构建目标名必须全局唯一（旧包也有 radar_node），但输出文件名保持 radar_node。
set_target_properties(ultrasonic_radar_modbus_tcp_node PROPERTIES
  OUTPUT_NAME radar_node
)

add_dependencies(ultrasonic_radar_modbus_tcp_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ultrasonic_radar_modbus_tcp_node
  ${catkin_LIBRARIES}
  Threads::Threads
)
