/**
 * @file chassis.cpp
 * @author qingyu
 * @brief 本项目的底盘节点入口：起一个 ChassisNode，把它和两个电机子节点一起 spin
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 节点本体在 chassis.hpp。跟 node/imu 的区别：那个是把通用 ImuNode 拿来直接用，
 *       这个是底盘，力矩怎么分给两个轮子是本项目的事，所以类就写在 project 这层
 * @note 用 SingleThreadedExecutor 而不是 rclcpp::spin(node)：一个进程里三个节点，
 *       子节点不加进执行器的话，它们的订阅和定时器一个都不会跑
 * @note 参数走 params 文件（params/chassis.yaml）：按节点名匹配，chassis 吃 /chassis 段，
 *       两个电机吃 ** 段（通配段，两轮共用一份，免得改漏一个轮子）和 /motor_left、
 *       /motor_right 段 —— 子节点也吃同一份文件，是因为它们的 NodeOptions
 *       没关 use_global_arguments
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "chassis.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto chassis = std::make_shared<ChassisNode>();

    // 底盘 + 两个电机：都在这个进程里，一起 spin
    rclcpp::executors::SingleThreadedExecutor executor;
    for (const auto& node : chassis->nodes())
    {
        executor.add_node(node);
    }
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
