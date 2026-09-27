/**
 * @file imu.hpp
 * @author qingyu
 * @brief IMU 解算模块：把原始角速度/加速度解成姿态，并整理成可直接发送的一帧数据
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include "framework/algorithm/filter/attitude/quaternion_attitude.hpp"

namespace modules::sensor {

/**
 * @brief 一帧原始 IMU 数据（输入契约）：node 在回调里填好，交给 Imu::update()
 */
struct ImuSample
{
    int64_t         stamp_ns{0};                                       // 采样时刻，纳秒；ROS 消息的 sec/nanosec 直接换算即可
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};      // 机体角速度，rad/s
    Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};   // 机体比力（含重力），m/s^2
};

/**
 * @brief 解算结果（输出契约）：node 直接拿去填要发的话题
 *
 * @note 含固定大小的 Eigen 类型（Quaterniond 32 字节，要求 16 字节对齐）。C++17 起
 *       aligned new 由编译器处理，不需要 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 */
struct ImuSolution
{
    int64_t             stamp_ns{0};                                       // 本帧采样时刻，纳秒，原样透传
    Eigen::Quaterniond  orientation{Eigen::Quaterniond::Identity()};       // 机体 -> 世界 姿态
    Eigen::Vector3d     euler_rpy_rad{Eigen::Vector3d::Zero()};         // 姿态的欧拉角形式；便于记录和调试，主输出是四元数
    Eigen::Vector3d     angular_velocity{Eigen::Vector3d::Zero()};      // 原始角速度，rad/s，原样透传
    Eigen::Vector3d     linear_acceleration{Eigen::Vector3d::Zero()};   // 原始比力，m/s^2，原样透传
    Eigen::Vector3d     up_body{Eigen::Vector3d::Zero()};               // 世界天向在机体系的分量（单位向量），供倾角判断/重力补偿
    bool                attitude_valid{false};                            // 姿态是否已有意义：第一帧拿不到 dt，不解算
};

/**
 * @brief IMU 解算器：持有姿态滤波器与上一帧时间戳，按帧推进
 *
 * @note 这一层只做解算和数据装配，不含任何通信/中间件代码：时间戳、话题、消息类型
 *       都由上层 node 决定；本模块只认纳秒时间戳和自己定义的两个结构体
 * @note 姿态用的是互补滤波（算法层 QuaternionAttitude），不是 EKF，所以给不出协方差。
 *       要填 sensor_msgs/Imu 的 orientation_covariance / angular_velocity_covariance /
 *       linear_acceleration_covariance，得由 node 决定填 0 还是首元素 -1（表示未提供）
 */
class Imu
{
public:
    using AttitudeFilter = algorithm::filter::QuaternionAttitude<double>;

    Imu() = default;

    /**
     * @brief 设置加速度修正增益：越大越信加速度计（姿态收敛快，但对机体自身加速度也更敏感）
     */
    void set_accel_correction_gain(double gain) { attitude_.set_accel_correction_gain(gain); }

    /**
     * @brief 清空姿态与时间戳，回到未初始化状态（复位后第一帧仍然只记时间戳）
     */
    void reset()
    {
        attitude_.reset();
        solution_        = ImuSolution{};
        last_stamp_ns_   = 0;
        has_last_stamp_  = false;
    }

    /**
     * @brief 推进一帧：算 dt、跑姿态滤波、刷新解算结果
     *
     * @param sample 本帧原始数据
     * @return bool true = 本帧出了新解，可以去读 solution()；false = 本帧被丢弃
     *
     * @note 丢帧的两种情况是有意的：第一帧（没有上一帧就算不出 dt）、时间戳没有前进
     *       （乱序或重复）。后者硬算成 dt<=0 只会让积分空转，却把这一帧的原始数据写进
     *       解里，不如直接不收
     */
    bool update(const ImuSample& sample)
    {
        // 第一帧只记时间戳：姿态积分需要 dt，此时还不知道
        if (!has_last_stamp_)
        {
            last_stamp_ns_  = sample.stamp_ns;
            has_last_stamp_ = true;
            return false;
        }

        if (sample.stamp_ns <= last_stamp_ns_)
        {
            return false;
        }

        const double dt_s = static_cast<double>(sample.stamp_ns - last_stamp_ns_) * 1e-9;
        last_stamp_ns_    = sample.stamp_ns;

        attitude_.update(sample.angular_velocity, sample.linear_acceleration, dt_s);

        solution_.stamp_ns            = sample.stamp_ns;
        solution_.orientation         = attitude_.quaternion();
        solution_.euler_rpy_rad       = attitude_.euler_rpy_rad();
        solution_.angular_velocity    = sample.angular_velocity;
        solution_.linear_acceleration = sample.linear_acceleration;
        solution_.up_body             = attitude_.up_body();
        solution_.attitude_valid      = true;
        return true;
    }

    const ImuSolution& solution() const { return solution_; }

    /**
     * @brief 是否已经出过至少一帧解
     */
    bool attitude_valid() const { return solution_.attitude_valid; }

    /**
     * @brief 姿态四元数，省得每次写 solution().orientation
     */
    Eigen::Quaterniond orientation() const { return solution_.orientation; }

    /**
     * @brief 姿态的欧拉角形式，省得每次写 solution().euler_rpy_rad
     */
    Eigen::Vector3d euler_rpy_rad() const { return solution_.euler_rpy_rad; }

private:
    AttitudeFilter  attitude_{};                    // 姿态互补滤波
    ImuSolution     solution_{};                    // 最新一帧解算结果
    int64_t         last_stamp_ns_{0};              // 上一帧时间戳
    bool            has_last_stamp_{false};         // 是否已经有上一帧
};

} // namespace modules::sensor
