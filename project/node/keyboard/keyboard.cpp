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
 * @note 发到哪条话题、松手多久算停、按键方向，都在 params/keyboard.yaml（install 里那份，
 *       下面自己找，找不到就用代码默认值）。临时改加 --ros-args -p，优先级更高
 * @note 发的是归一化方向（±1 = 满速），不是 m/s —— 实际多快归底盘，见 params/chassis.yaml
 * @note 必须在有终端的窗口里跑：它读的是本进程的 stdin
 */

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_layer/node/keyboard/keyboard.hpp"

int main(int argc, char** argv)
{
    // 参数文件是**全局**参数，只能走命令行：塞进 NodeOptions::arguments 的话 rcl 不认，
    // 会一声不吭地忽略掉（节点照跑，参数全是代码默认值）
    std::vector<std::string> arg_strings(argv, argv + argc);
    const std::string params_file = ament_index_cpp::get_package_share_directory("project") + "/params/keyboard.yaml";

    if (std::filesystem::exists(params_file))
    {
        arg_strings.push_back("--ros-args");
        arg_strings.push_back("--params-file");
        arg_strings.push_back(params_file);
    }

    std::vector<char const*> args;
    args.reserve(arg_strings.size());

    for (const auto& arg : arg_strings)
    {
        args.push_back(arg.c_str());
    }

    rclcpp::init(static_cast<int>(args.size()), args.data());

    // 自己的窗口里前台跑，别塞进 launch 的后台：读的是这个进程的终端
    rclcpp::spin(std::make_shared<KeyboardNode>());

    // 退出时节点析构 -> Keyboard 析构 -> 终端设置恢复
    rclcpp::shutdown();

    return 0;
}
