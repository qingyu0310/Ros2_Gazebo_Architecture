/**
 * @file leg.cpp
 * @author qingyu
 * @brief 本项目的腿姿保持器入口：起一个 LegNode，管四个腿关节
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 * @note 节点本体在 leg.hpp。跟 node/motor 的区别：那个是通用执行器（谁都能用），
 *       这个只管"轮腿车的腿该出多大力"，所以类就写在 project 这层
 * @note 参数走 params 文件（params/chassis.yaml），按节点名匹配 /leg 段
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "leg.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LegNode>());

    rclcpp::shutdown();

    return 0;
}
