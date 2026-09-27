/**
 * @file chassis.cpp
 * @author qingyu
 * @brief 本项目的底盘节点入口：起一个 ChassisNode，只管解算
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 节点本体在 chassis.hpp。跟 node/imu 的区别：那个是把通用 ImuNode 拿来直接用，
 *       这个是底盘，力矩怎么分给两个轮子是本项目的事，所以类就写在 project 这层
 * @note 电机不在这里：拆到 node/motor 那个进程了，两边过 /motor/<轮子>/command 这一路话题
 * @note 参数走 params 文件（params/chassis.yaml），按节点名匹配 /chassis 段
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "chassis.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ChassisNode>());

    rclcpp::shutdown();

    return 0;
}
