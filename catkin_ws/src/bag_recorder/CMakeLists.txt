cmake_minimum_required(VERSION 3.0.2)
project(bag_recorder)

# 设置 C++17 标准，并强制要求编译器支持
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找 catkin 和相关依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs      
  sensor_msgs
  roslib       # 确保这里声明了 roslib
)

## 声明 catkin 包信息
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    std_srvs 
    sensor_msgs
    roslib
)

###########
## Build ##
###########

## 指定头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 声明可执行文件
## main.cpp：参数读取与 ROS 拓扑；bag_recorder_node.cpp：录制业务流水线
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/bag_recorder_node.cpp
)

## 添加依赖（确保消息等生成后再编译）
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 链接库文件
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${rospack_LIBRARIES}
)

#############
## Install ##
#############

## 安装可执行文件
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 如果有 launch 文件或其他资源，可以取消下面的注释进行安装
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

#############
## Testing ##
#############

## 如果需要添加 gtest，可以取消下面的注释
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bag_recorder.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()