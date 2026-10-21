cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

# 本包只承载底盘/上装控制的消息定义，不含可执行节点。
# 消息被 perception_sensors_client_main 等节点以 common_msgs/<消息名> 引用。

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ChassisMsg.msg
  ChassisCtrl.msg
  TopMsg.msg
)

# ChassisMsg / TopMsg 的 Header 字段依赖 std_msgs，
# ChassisMsg 还内嵌同包的 ChassisCtrl、TopMsg（同包引用无需声明）。
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
