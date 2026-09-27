/**
 * @file motor.hpp
 * @author qingyu
 * @brief 电机节点：收目标值（力矩或速度）和关节反馈，交给 framework 的 modules::motor::Motor 整形，把力矩发给执行器
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 写成头文件而不是 .cpp：类定义连实现全 inline，project 那层直接
 *       #include "ros2_layer/node/motor/motor.hpp" 然后 make_shared<MotorNode>()。
 *       所以这个包做成 INTERFACE 库，不产二进制（见本包 CMakeLists.txt）
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "framework/modules/motors/motor.hpp"

/**
 * @brief 单关节电机节点：只做 ROS 侧收发，限幅/死区/速度环全在 modules::motor::Motor 里
 *
 * @note 一个实例管一个关节：左轮右轮各起一个，joint_name 不同
 * @note 本文件不含 main：这里只定义节点类，入口在 project 那层，那边
 *       rclcpp::spin(std::make_shared<MotorNode>()) 就起来了
 * @note 一个进程里要起多个电机（比如四个轮子）时，用带节点名的构造函数把它们区分开：
 *           std::make_shared<MotorNode>("motor_front_left")
 *       节点名不同，参数和话题才是各管各的：params 文件里按这些名字分组给各自的
 *       joint_name / force_topic / 限幅 / PI。名字都叫 "motor" 的话，一条
 *       --ros-args -p joint_name:=... 会同时套到四个节点上
 * @note 父节点（底盘）可以把子节点该用哪套配置通过 NodeOptions::parameter_overrides 传进来，
 *       见 project/node/chassis/chassis.hpp
 * @note 类名带 Node 是为了跟解算模块 modules::motor::Motor 区分开
 * @note 发出去的是 std_msgs/Float64，由 launch 里的桥接到 gz 的 cmd_force（gz.msgs.Double）。
 *       gz 那边话题名里焊死了模型名（/model/<模型名>/joint/<关节名>/cmd_force），所以
 *       ROS 侧叫什么名字、怎么桥，都是 launch 的事，本节点只认自己的参数
 */
class MotorNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造一个单关节电机节点
     *
     * @param node_name 节点名。一个进程里只起一个电机时可以不管；起多个时必须各不相同，
     *                  否则它们的参数会互相串
     * @param options 节点选项，用来把本节点该用的参数直接塞进来（底盘给自己的四个子节点用）
     */
    explicit MotorNode(const std::string& node_name = "motor", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options)
    {
        joint_name_         = declare_parameter("joint_name", "joint");
        command_topic_      = declare_parameter("command_topic", "/motor/command");
        force_topic_        = declare_parameter("force_topic", "/motor/cmd_force");
        joint_states_topic_ = declare_parameter("joint_states_topic", "/joint_states");
        publish_period_     = declare_parameter("publish_period_s", 0.001);

        mode_ = parse_mode(declare_parameter("control_mode", "effort"));

        motor_.set_mode(mode_);
        motor_.set_effort_limit(declare_parameter("max_effort", 5.0));
        motor_.set_speed_limit(declare_parameter("max_speed_radps", 60.0));
        motor_.set_speed_gains(declare_parameter("speed_kp", 0.10), declare_parameter("speed_ki", 1.0));
        motor_.set_command_timeout(declare_parameter("effort_timeout_s", 0.05));

        // 回调只缓存最新值，整形统一放在定时器里做，跟话题频率解耦
        command_sub_ = create_subscription<std_msgs::msg::Float64>(command_topic_, 10, std::bind(&MotorNode::on_command, this, std::placeholders::_1));

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(joint_states_topic_, rclcpp::SensorDataQoS(), std::bind(&MotorNode::on_joint_state, this, std::placeholders::_1));

        force_pub_ = create_publisher<std_msgs::msg::Float64>(force_topic_, 10);

        timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_), std::bind(&MotorNode::update, this));
    }

private:
    std::string joint_name_;
    std::string command_topic_;
    std::string force_topic_;
    std::string joint_states_topic_;
    double      publish_period_{0.001};

    modules::motor::Mode  mode_{modules::motor::Mode::kEffort};
    modules::motor::Motor motor_;                 // 限幅/死区/速度环全在这里，节点不碰

    double  target_command_{0.0};                 // 最新一条目标值：力矩模式下是 N·m，速度模式下是 rad/s
    bool    has_command_{false};                  // 收到过指令没有
    std::chrono::steady_clock::time_point last_command_time_{};

    double  joint_speed_radps_{0.0};              // 这个关节的反馈角速度
    bool    has_joint_feedback_{false};           // /joint_states 的这一帧里有没有它

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr       command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          force_pub_;

    /**
     * @brief 把 control_mode 参数翻成模块的枚举
     *
     * @param mode 参数值，"effort" 或 "speed"
     * @return modules::motor::Mode 控制模式
     *
     * @throw std::invalid_argument 参数不是这两个之一时抛，别让它静默跑成力矩模式
     */
    static modules::motor::Mode parse_mode(const std::string& mode)
    {
        if (mode == "effort")
        {
            return modules::motor::Mode::kEffort;
        }
        if (mode == "speed")
        {
            return modules::motor::Mode::kSpeed;
        }
        throw std::invalid_argument("control_mode must be 'effort' or 'speed'");
    }

    /**
     * @brief 收到一条目标值：只缓存，整形留给定时器那一拍
     *
     * @param msg 目标值。力矩模式下是力矩 N·m，速度模式下是角速度 rad/s，由 control_mode 决定
     *
     * @note 这里不做限幅：夹紧是模块的活，夹两遍只会让"到底谁夹的"说不清
     */
    void on_command(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_command_    = msg->data;
        has_command_       = true;
        last_command_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 收到一帧关节状态：挑出这个关节的角速度
     *
     * @param msg 关节状态
     *
     * @note 这一帧里没有本关节就把反馈标成无效：宁可让模块这拍输出 0，也别拿上一帧的旧速度
     *       当现值。关节名写错时也必须看得见（输出一直是 0 + 日志），不能装成"速度一直是 0"
     */
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        has_joint_feedback_ = false;

        // name 和 velocity 不保证一样长，一起截短
        const std::size_t count = std::min(msg->name.size(), msg->velocity.size());
        for (std::size_t i = 0; i < count; ++i)
        {
            if (msg->name[i] == joint_name_)
            {
                joint_speed_radps_  = msg->velocity[i];
                has_joint_feedback_ = true;
                return;
            }
        }
    }

    /**
     * @brief 最新一条指令到现在过了多久
     *
     * @return double 秒；一条指令都没收到过时返回无穷大，模块那边判成"不新鲜"
     *
     * @note 用 steady_clock 而不是仿真时间：仿真会被暂停/复位，拿它算"多久没收到指令"会判错
     */
    double command_age_s() const
    {
        if (!has_command_)
        {
            return std::numeric_limits<double>::infinity();
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_time_).count();
    }

    /**
     * @brief 定时器里跑的一拍：把最新指令和关节反馈喂给模块，把算出来的力矩发出去
     *
     * @note 时间戳用 now()（跟着 use_sim_time 走），给速度环算 dt 用
     */
    void update()
    {
        modules::motor::MotorTick tick;
        tick.stamp_ns           = now().nanoseconds();
        tick.command_age_s      = command_age_s();
        tick.speed_radps        = joint_speed_radps_;
        tick.has_speed_feedback = has_joint_feedback_;

        // 一条话题，值的意思由模式决定：只填这个模式下该读的那个
        if (mode_ == modules::motor::Mode::kEffort)
        {
            tick.target_effort_nm = target_command_;
        }
        else
        {
            tick.target_speed_radps = target_command_;
        }

        motor_.update(tick);

        std_msgs::msg::Float64 force;
        force.data = motor_.effort_nm();
        force_pub_->publish(force);

        // 出力被压成 0（指令超时 / 速度模式还没有关节反馈）时留个痕，
        // 不然"为什么不转"只能靠猜。节流到 1 s 一条
        if (!motor_.output().active)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "关节 %s 这拍没出力：指令超时或没有关节反馈，力矩按 0 发",
                                 joint_name_.c_str());
        }
    }
};
