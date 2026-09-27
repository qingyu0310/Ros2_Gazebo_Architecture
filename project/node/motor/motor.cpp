/**
 * @file motor.cpp
 * @author qingyu
 * @brief 本项目的电机节点入口：六个执行器各起一个 MotorNode，跟底盘分开进程跑
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 * @note 节点本体是 ros2_layer 的通用 MotorNode，跟本项目的机器无关，所以那边一条
 *       .cpp 都没有，只留头文件；这里只负责"这台车有几个关节、都叫什么名字"
 * @note 六个节点名必须各不相同（motor_left / motor_right / motor_left_hip /
 *       motor_right_hip / motor_left_knee / motor_right_knee）：params 文件按节点名分段，
 *       名字一样的话参数会互相串。六段见 params/chassis.yaml
 * @note 接线（关节名 / 指令话题 / 力矩话题）全在 params 文件里，这里不写死。
 *       顺序 [左轮, 右轮, 左胯, 右胯, 左膝, 右膝] 只是读起来顺，节点之间互不相干
 */

#include <array>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_layer/node/motor/motor.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    constexpr std::size_t kMotorCount = 6;
    const std::array<std::string, kMotorCount> kMotorNames{"motor_left",      "motor_right",
                                                           "motor_left_hip",  "motor_right_hip",
                                                           "motor_left_knee", "motor_right_knee"};

    // 节点得自己接住：executor 只存弱引用，循环里造完就扔的话节点当场析构 ——
    // 进程还活着，图里却一个节点都没有，而且一声不吭
    std::array<std::shared_ptr<MotorNode>, kMotorCount> motors{};
    rclcpp::executors::SingleThreadedExecutor executor;

    for (std::size_t i = 0; i < kMotorCount; ++i)
    {
        motors[i] = std::make_shared<MotorNode>(kMotorNames[i]);
        executor.add_node(motors[i]);
    }

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
