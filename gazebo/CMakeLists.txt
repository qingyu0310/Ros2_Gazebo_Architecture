cmake_minimum_required(VERSION 3.8)
project(gazebo)

find_package(ament_cmake REQUIRED)

# 通用资源：世界文件、图元模型、插件宏，按目录原样装进 share/
install(DIRECTORY models plugins worlds
    DESTINATION share/${PROJECT_NAME})

ament_package()
