cmake_minimum_required(VERSION 3.8)
project(project)

find_package(ament_cmake REQUIRED)
# 不写 find_package(Eigen3)：Eigen 由 framework 经 ament_export_dependencies 透传上来
find_package(framework REQUIRED)
# ros2_layer 的通用节点（ImuNode 等）从这里来
find_package(ros2_layer REQUIRED)

# 节点可执行目标加在这里（node/ 是源码，不进 install）：
#   add_executable(<node> node/<方向>/<文件>.cpp)
#   ament_target_dependencies(<node> rclcpp ...)
#   target_link_libraries(<node> framework::framework)
#   install(TARGETS <node> DESTINATION lib/${PROJECT_NAME})

# 链 ros2_layer::ros2_layer 就够了：rclcpp / 消息包 / framework（含 Eigen）都由它透传下来。
# 要单独用 framework 的算法头时再写 framework::framework（上面已经 find_package 过）。
add_executable(imu node/imu/imu.cpp)
target_link_libraries(imu ros2_layer::ros2_layer)
install(TARGETS imu DESTINATION lib/${PROJECT_NAME})

# 底盘：一个进程里三个节点（chassis + 两个电机），电机由底盘一起 spin
add_executable(chassis node/chassis/chassis.cpp)
target_link_libraries(chassis ros2_layer::ros2_layer)
install(TARGETS chassis DESTINATION lib/${PROJECT_NAME})

# 键盘遥控：读终端方向键，往底盘的 cmd_vel 发 Twist。要自己在有终端的窗口里前台跑
add_executable(keyboard node/keyboard/keyboard.cpp)
target_link_libraries(keyboard ros2_layer::ros2_layer)
install(TARGETS keyboard DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params robot
    DESTINATION share/${PROJECT_NAME})

ament_package()
