cmake_minimum_required(VERSION 3.10)
project(main_control)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  raicom_vision_laser
)

catkin_package(
  CATKIN_DEPENDS raicom_vision_laser 
)

# 包含头文件目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# 收集所有源文件
file(GLOB SOURCES "src/*.cpp" "src/*.c")
# cpp compile part
add_executable(main_control src/main_control.cpp)
target_link_libraries(main_control ${catkin_LIBRARIES})
add_dependencies(main_control ${catkin_EXPORTED_TARGETS})  # 确保消息生成
