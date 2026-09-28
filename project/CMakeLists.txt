cmake_minimum_required(VERSION 3.8)
project(project)

find_package(ament_cmake REQUIRED)
# 不写 find_package(Eigen3)：Eigen 由 framework 经 ament_export_dependencies 透传上来
find_package(framework REQUIRED)
# ros2_layer 的通用节点（ImuNode 等）从这里来
find_package(ros2_layer REQUIRED)
# 键盘节点用它找 share/project/params/keyboard.yaml
find_package(ament_index_cpp REQUIRED)
# 离线工具（generate_wheel_leg_lqr）读展开的 URDF 用它
find_package(tinyxml2_vendor REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

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

# 底盘：只管解算，算完把两个轮子的力矩发到话题上
add_executable(chassis node/chassis/chassis.cpp)
target_link_libraries(chassis ros2_layer::ros2_layer ament_index_cpp::ament_index_cpp yaml-cpp)
install(TARGETS chassis DESTINATION lib/${PROJECT_NAME})

# 电机：两个轮子各一个 MotorNode，跟底盘分开进程。以后髋、膝加进来也起在这个进程里
add_executable(motor node/motor/motor.cpp)
target_link_libraries(motor ros2_layer::ros2_layer)
install(TARGETS motor DESTINATION lib/${PROJECT_NAME})

# 离线工具：从展开的 URDF 建平面共模模型，线性化 A/B，解 K，写 params/leg_gain/ 下的增益文件。
# 不在仿真链路上，跑一次生成一次；读 URDF 用 tinyxml2（ros2 自带）
add_executable(generate_wheel_leg_lqr tools/generate_wheel_leg_lqr.cpp)
target_link_libraries(generate_wheel_leg_lqr framework::framework tinyxml2::tinyxml2)
install(TARGETS generate_wheel_leg_lqr DESTINATION lib/${PROJECT_NAME})

# 键盘遥控：读终端方向键，往底盘的 cmd_vel 发 Twist。要自己在有终端的窗口里前台跑
add_executable(keyboard node/keyboard/keyboard.cpp)
target_link_libraries(keyboard ros2_layer::ros2_layer ament_index_cpp::ament_index_cpp)
install(TARGETS keyboard DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params robot
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_torque_allocator test/test_torque_allocator.cpp)
    target_include_directories(test_torque_allocator PRIVATE node/chassis)

    ament_add_gtest(test_leg_height test/test_leg_height.cpp)
    target_include_directories(test_leg_height PRIVATE node/chassis)

    ament_add_gtest(test_runtime_lqr test/test_runtime_lqr.cpp)
    target_include_directories(test_runtime_lqr PRIVATE node/chassis)
    target_link_libraries(test_runtime_lqr framework::framework)

    ament_add_gtest(test_chassis_modules test/test_chassis_modules.cpp)
    target_include_directories(test_chassis_modules PRIVATE node/chassis)
    target_link_libraries(test_chassis_modules framework::framework yaml-cpp)
endif()

ament_package()
