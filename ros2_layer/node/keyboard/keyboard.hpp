/**
 * @file keyboard.hpp
 * @author qingyu
 * @brief 键盘遥控节点：读终端方向键，算出底盘速度发出去
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 写成头文件而不是 .cpp：类定义连实现全 inline，project 那层直接
 *       #include "ros2_layer/node/keyboard/keyboard.hpp" 然后 make_shared<KeyboardNode>()。
 *       所以这个包做成 INTERFACE 库，不产二进制（见本包 CMakeLists.txt）
 *
 * @note 本节点只发不收：一条 Twist 出去（默认 /chassis/cmd_vel），底盘节点在那边收。
 *       键盘读的是本进程的 stdin，所以它得在**你自己的终端**里跑，不能塞进 launch 的
 *       后台日志里 —— 没有 tty 的话构造就失败，会直接告诉你
 *
 * @note 默认 1000 Hz 重发（publish_period_s=0.001），跟底盘、电机的节拍对齐：
 *       松手之后也得持续发 0，不能"停了不发"，否则底盘那边只能靠自己的超时兜
 */

#pragma once

#include <chrono>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "framework/modules/keyboard/keyboard.hpp"

/**
 * @brief 键盘遥控节点：方向键 -> 底盘速度
 *
 * @note 键位和松手判定都在 modules::keyboard::Teleop 里，本节点只做三件事：
 *       读键、算按键年龄（steady_clock）、发 Twist
 */
class KeyboardNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造一个键盘遥控节点
     *
     * @param node_name 节点名
     * @param options 节点选项
     *
     * @throw std::runtime_error stdin 不是终端时抛（stdin 被重定向、或者跑在没有 tty 的后台里）
     */
    explicit KeyboardNode(const std::string& node_name = "keyboard", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options)
    {
        cmd_vel_topic_     = declare_parameter("cmd_vel_topic",         "/chassis/cmd_vel");
        publish_period_    = declare_parameter("publish_period_s",      0.001);
        key_timeout_       = declare_parameter("key_timeout_s",         0.5);
        linear_speed_      = declare_parameter("linear_speed_mps",      0.5);
        angular_speed_     = declare_parameter("angular_speed_radps",   1.0);

        teleop_.set_linear_speed(linear_speed_);
        teleop_.set_angular_speed(angular_speed_);
        teleop_.set_key_timeout(key_timeout_);

        // 终端切不过去就没得玩：早报早好，别让一个收不到键的节点在那儿发 0
        if (!keyboard_.open())
        {
            RCLCPP_FATAL(get_logger(), "读不到终端：stdin 不是 tty（被重定向了？）或者没有权限");
            throw std::runtime_error("keyboard: stdin is not a tty");
        }

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        timer_       = create_wall_timer(std::chrono::duration<double>(publish_period_),
                                         std::bind(&KeyboardNode::update, this));

        RCLCPP_INFO(get_logger(), "键盘就绪：方向键驾驶，松手 %.2f s 后停，Ctrl-C 退出（发到 %s）", key_timeout_, cmd_vel_topic_.c_str());
    }

private:
    modules::keyboard::Keyboard keyboard_;   // 析构时恢复终端设置，别让 shell 卡在原始模式
    modules::keyboard::Teleop   teleop_;

    std::string cmd_vel_topic_;
    double      publish_period_ {0.001};
    double      key_timeout_    {0.7};
    double      linear_speed_   {1.0};
    double      angular_speed_  {2.0};

    bool                                  has_key_{false};
    std::chrono::steady_clock::time_point last_key_time_{};

    rclcpp::TimerBase::SharedPtr                              timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_pub_;

    /**
     * @brief 最后一次按键到现在过了多久
     *
     * @return double 秒；一次都没按过时返回无穷大，模块那边判成"不新鲜"
     *
     * @note 跟 MotorNode 一样用 steady_clock：仿真会暂停/复位，拿仿真时间算"多久没按键"会判错。
     *       再说键盘本来就是真实时间上的东西，跟仿真时间没关系
     */
    double key_age_s() const
    {
        if (!has_key_)
        {
            return std::numeric_limits<double>::infinity();
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_key_time_).count();
    }

    /**
     * @brief 定时器里跑的一拍：读键 -> 喂给遥控模块 -> 把速度发出去
     *
     * @note 没按键的时候也照发（发的是 0）：底盘那边只看"指令新不新鲜"，这边一发 0，
     *       车就是被指令刹停的，而不是靠超时兜底
     */
    void update()
    {
        modules::keyboard::KeyTick tick;
        tick.key = keyboard_.poll();

        if (tick.key != modules::keyboard::Key::kNone)
        {
            has_key_       = true;
            last_key_time_ = std::chrono::steady_clock::now();
            RCLCPP_DEBUG(get_logger(), "按键：%u", static_cast<unsigned>(tick.key));
        }

        // 年龄要在刷新时间戳之后算：刚按下的这一拍年龄是 0，才是"新鲜"
        tick.key_age_s = key_age_s();
        teleop_.update(tick);

        geometry_msgs::msg::Twist msg;
        msg.linear.x  = teleop_.output().linear_mps;
        msg.angular.z = teleop_.output().angular_radps;
        cmd_vel_pub_->publish(msg);
    }
};
