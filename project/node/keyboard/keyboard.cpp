/**
 * @file keyboard.cpp
 * @author qingyu
 * @brief 本项目的键盘遥控节点入口：直接用 ros2 层的 KeyboardNode，不重复写读键逻辑
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这里只有入口。节点本体（参数、定时器、发布）在 ros2_layer 包的
 *       ros2_layer/node/keyboard/keyboard.hpp，跟本项目的机器无关，所以放通用层；
 *       键位到速度的映射在 framework 的 modules/keyboard/keyboard.hpp
 * @note 发到哪条话题、速度多大、松手多久算停，都是 KeyboardNode 的参数：
 *       cmd_vel_topic / linear_speed_mps / angular_speed_radps / key_timeout_s，
 *       下面用默认构造，--ros-args -p 或 params 文件照常生效
 * @note 必须在有终端的窗口里跑：它读的是本进程的 stdin
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ros2_layer/node/keyboard/keyboard.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 自己的窗口里前台跑，别塞进 launch 的后台：读的是这个进程的终端
    rclcpp::spin(std::make_shared<KeyboardNode>());

    // 退出时节点析构 -> Keyboard 析构 -> 终端设置恢复
    rclcpp::shutdown();

    return 0;
}
