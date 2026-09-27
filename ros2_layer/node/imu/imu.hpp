/**
 * @file imu.hpp
 * @author qingyu
 * @brief IMU 节点：收原始 IMU，交给 framework 的 modules::sensor::Imu 解算，再把结果发出去
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 写成头文件而不是 .cpp：类定义连实现全 inline，project 那层直接
 *       #include "ros2_layer/node/imu/imu.hpp" 然后 make_shared<ImuNode>()。
 *       所以这个包做成 INTERFACE 库，不产二进制（见本包 CMakeLists.txt）
 */

#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "framework/modules/sensors/imu.hpp"

/**
 * @brief IMU 解算节点：只做 ROS 侧收发，解算全在 modules::sensor::Imu 里
 *
 * @note 本文件不含 main：这里只定义节点类，入口在 project 那层，那边
 *       rclcpp::spin(std::make_shared<ImuNode>()) 就起来了
 * @note 类名带 Node 是为了跟解算模块 modules::sensor::Imu 区分开
 */
class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : rclcpp::Node("imu")
    {
        raw_topic_      = declare_parameter("raw_topic", "/imu");
        frame_id_       = declare_parameter("frame_id", "base_link");
        publish_period_ = declare_parameter("publish_period_s", 0.001);

        imu_module_.set_accel_correction_gain(declare_parameter("accel_correction_gain", 0.1));

        // 回调只缓存最新一帧，解算统一放在定时器里做，跟原始 IMU 的频率解耦
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(raw_topic_, rclcpp::SensorDataQoS(), std::bind(&ImuNode::on_imu, this, std::placeholders::_1));

        attitude_pub_            = create_publisher<sensor_msgs::msg::Imu>("/imu/attitude", 10);
        quaternion_pub_          = create_publisher<geometry_msgs::msg::QuaternionStamped>("/imu/quaternion", 10);
        euler_rad_pub_           = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/euler_rad", 10);
        euler_deg_pub_           = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/euler_deg", 10);
        angular_velocity_pub_    = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/angular_velocity", 10);
        linear_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/linear_acceleration", 10);

        timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_), std::bind(&ImuNode::update, this));
    }

private:
    std::string raw_topic_;
    std::string frame_id_;
    double      publish_period_{0.001};

    modules::sensor::Imu  imu_module_;              // 解算全在这里，节点不碰滤波
    sensor_msgs::msg::Imu latest_raw_;              // 最新一帧原始消息
    bool                  new_raw_{false};          // 有没有还没处理的新帧

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                 attitude_pub_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr    euler_rad_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr    euler_deg_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr    angular_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr    linear_acceleration_pub_;

    /**
     * @brief 收到一帧原始 IMU：只缓存，解算留给定时器那一拍
     *
     * @param msg 原始 IMU 消息
     */
    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_raw_ = *msg;
        new_raw_    = true;
    }

    /**
     * @brief 定时器里跑的一拍：把最新原始帧喂给解算模块，出新解就发出去
     */
    void update()
    {
        if (!new_raw_)
        {
            return;
        }
        new_raw_ = false;

        modules::sensor::ImuSample sample;
        sample.stamp_ns            = stamp_to_ns(latest_raw_.header.stamp);
        sample.angular_velocity    = Eigen::Vector3d(latest_raw_.angular_velocity.x, latest_raw_.angular_velocity.y, latest_raw_.angular_velocity.z);
        sample.linear_acceleration = Eigen::Vector3d(latest_raw_.linear_acceleration.x, latest_raw_.linear_acceleration.y, latest_raw_.linear_acceleration.z);

        // 模块自己按两帧时间戳算 dt；第一帧和解算不出 dt 的帧会被它丢掉
        if (!imu_module_.update(sample))
        {
            return;
        }

        const auto& solution = imu_module_.solution();
        publish_attitude(solution);
        publish_quaternion(solution);
        publish_euler(solution);
        publish_vector(angular_velocity_pub_, solution.angular_velocity);
        publish_vector(linear_acceleration_pub_, solution.linear_acceleration);
    }

    /**
     * @brief 原始 IMU 消息透传，只把 header 和 orientation 换成本地解算结果
     *
     * @param solution 本帧解算结果
     */
    void publish_attitude(const modules::sensor::ImuSolution& solution)
    {
        auto msg = latest_raw_;
        msg.header.frame_id = frame_id_;

        const Eigen::Quaterniond& q = solution.orientation;
        msg.orientation.w = q.w();
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();

        attitude_pub_->publish(msg);
    }

    /**
     * @brief 只发姿态四元数
     *
     * @param solution 本帧解算结果
     */
    void publish_quaternion(const modules::sensor::ImuSolution& solution)
    {
        const Eigen::Quaterniond& q = solution.orientation;

        geometry_msgs::msg::QuaternionStamped msg;
        msg.header.stamp    = latest_raw_.header.stamp;
        msg.header.frame_id = frame_id_;
        msg.quaternion.w    = q.w();
        msg.quaternion.x    = q.x();
        msg.quaternion.y    = q.y();
        msg.quaternion.z    = q.z();

        quaternion_pub_->publish(msg);
    }

    /**
     * @brief 姿态的欧拉角，弧度和角度各发一条
     *
     * @param solution 本帧解算结果
     */
    void publish_euler(const modules::sensor::ImuSolution& solution)
    {
        const Eigen::Vector3d& euler_rad = solution.euler_rpy_rad;
        const Eigen::Vector3d  euler_deg = euler_rad * (180.0 / std::acos(-1.0));

        publish_vector(euler_rad_pub_, euler_rad);
        publish_vector(euler_deg_pub_, euler_deg);
    }

    /**
     * @brief 把一个三维向量打包成 Vector3Stamped 发出去
     *
     * @param publisher 目标话题
     * @param value 要发的向量
     *
     * @note 时间戳统一取原始消息的，避免 ns 再转回 ROS 时间
     */
    void publish_vector(const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr& publisher, const Eigen::Vector3d& value)
    {
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp    = latest_raw_.header.stamp;
        msg.header.frame_id = frame_id_;
        msg.vector.x        = value.x();
        msg.vector.y        = value.y();
        msg.vector.z        = value.z();

        publisher->publish(msg);
    }

    /**
     * @brief ROS 时间转纳秒，喂给解算模块用
     *
     * @param stamp ROS 消息里的时间戳
     * @return int64_t 纳秒
     */
    static int64_t stamp_to_ns(const builtin_interfaces::msg::Time& stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
    }
};
