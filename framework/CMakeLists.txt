cmake_minimum_required(VERSION 3.8)
project(framework)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# 纯模板头，没有 .cpp，所以做成 INTERFACE 库：本包不产二进制，只导出 include 路径和 Eigen
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

# 装成 include/framework/algorithm/... 和 include/framework/modules/...，配合下面的
# ament_export_include_directories(include)，消费方写 #include "framework/modules/sensor/imu.hpp"。
# 这两处必须配对，只改一处就找不到头文件。
# FILES_MATCHING 只挑 .hpp：modules/sensor/ 下的 imu.cpp 是空占位，不会装出去
install(DIRECTORY algorithm modules
    DESTINATION include/framework
    FILES_MATCHING PATTERN "*.hpp")

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_targets)
install(EXPORT ${PROJECT_NAME}_targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_targets)
ament_export_dependencies(Eigen3)

ament_package()
