cmake_minimum_required(VERSION 3.8)
project(ros2_layer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(framework REQUIRED)

# 节点写成"带实现的头文件"（类定义连函数实现全在 imu.hpp 里 inline），本包不产二进制，
# 所以跟 framework 一样做成 INTERFACE 库：只导出 include 路径 + 依赖，消费方（project）写
#   #include "ros2_layer/node/imu/imu.hpp"
#   rclcpp::spin(std::make_shared<ImuNode>());
# rclcpp / 消息包 / framework（连着 Eigen）都从这里透传下去，project 不用自己 find_package。
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME} INTERFACE
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    framework::framework)

# 装成 include/ros2_layer/node/imu/imu.hpp，配合下面的 ament_export_include_directories(include)，
# 消费方才能写 #include "ros2_layer/node/imu/imu.hpp"。这两处必须配对，只改一处就找不到头文件。
# FILES_MATCHING 只挑 .hpp：本包节点都是头文件形式，将来真加了带 .cpp 的可执行目标
# （add_executable + install(TARGETS ... DESTINATION lib/${PROJECT_NAME})），那部分不走这里。
install(DIRECTORY node
    DESTINATION include/ros2_layer
    FILES_MATCHING PATTERN "*.hpp")

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_targets)
install(EXPORT ${PROJECT_NAME}_targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_targets)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs std_msgs framework)

ament_package()
