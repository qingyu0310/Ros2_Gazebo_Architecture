/**
 * @file imu.cpp
 * @author qingyu
 * @brief 本项目的 IMU 节点入口：直接用 ros2 层的 ImuNode，不重复写收发逻辑
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这里只有入口。节点本体（话题、参数、定时器）在 ros2_layer 包的
 *       ros2_layer/node/imu/imu.hpp，跟本项目的机器无关，所以放通用层。
 * @note 本项目要改话题名/频率/增益，走 launch 或 params 文件传参即可：
 *       ImuNode 把 raw_topic / frame_id / publish_period_s / accel_correction_gain
 *       都声明成了参数，下面用默认构造，全局参数（--ros-args -p 或 parameters=）
 *       会照常生效，不用在这里再写一遍。
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ros2_layer/node/imu/imu.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ImuNode>());

    rclcpp::shutdown();

    return 0;
}
