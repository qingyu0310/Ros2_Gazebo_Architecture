/**
 * @file leg.hpp
 * @author qingyu
 * @brief 腿姿保持器：把四个腿关节顶在标准姿态上 —— 静态重力前馈 U0 + 关节 PD
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这是文档 §8 阶段 2 的临时基线：只解决"腿在自重下折下去"，车身的平衡还是
 *       node/chassis 那条轮式平衡环。全身 LQR（阶段 3）上来之后这个节点直接换掉
 * @note 话题：订 /joint_states；发 /motor/<关节>/command（N·m，一个关节一条），
 *       轮子那两路归 chassis，两边不重叠。电机在 node/motor 那个进程里，只管限幅
 * @note 这里不管的事：没有 IMU、不读高度指令、没有行程软限位、没有倒地保护 ——
 *       那些是阶段 4/5 的活。这个节点只有"给定目标腿姿 -> 出四个关节力矩"这一段
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

/**
 * @brief 腿几何与质量：算静态重力前馈 U0 用的量，跟 modules/leg.xacro、modules/wheel.xacro 一份数
 *
 * @note 这几个数跟 chassis 那套模型参数同源（body_mass 就是那边的 body_mass），模型改了
 *       这里要跟着改 —— 对不上不报错，只是腿会慢慢歪到别处去
 */
struct LegGeometry
{
    double leg_length {0.056};          // L，单根连杆长（大腿小腿同款）
    double leg_angle  {0.6981317008};   // 与水平面的夹角 40°，标准姿态即 α=50°、β=80°、胯到轮轴 72mm
    double body_mass  {1.25};           // 除两个轮子外的整车质量（已含四根连杆和膝上圆柱）
    double rod_mass   {0.03};           // 单根连杆
    double joint_mass {0.005};          // 膝上的关节圆柱
    double wheel_mass {0.15};           // 单个轮子
    double gravity    {9.81};
};

/**
 * @brief 保持参数：目标腿姿、PD 增益、出力上限、符号
 *
 * @note 目标腿姿是**关节角**，不是高度：标准姿态就是关节零位，所以两条腿都是 0。
 *       等阶段 6 要按高度走的时候，这里换成"高度 -> 关节角"的解算
 */
struct HoldParams
{
    double dt                 {0.001};                  // 控制周期，跟平衡环、物理步长一个节奏
    double target_hip_rad     {0.0};                    // 目标胯角 rad，两条腿一个值
    double target_knee_rad    {0.0};                    // 目标膝角 rad
    double kp                 {2.0};                    // N·m/rad，起步值抄 lqr 那台的 leg 节点
    double kd                 {0.05};                   // N·m·s/rad
    double max_effort         {1.0};                    // 单关节出力上限 N·m，模型的硬限位是 1.5
    double feedback_timeout_s {0.10};                   // 这么久没收到 /joint_states 就归零
    std::vector<double> joint_sign {1.0, 1.0, 1.0, 1.0};   // 正方向反了翻成 -1.0，顺序同下
};

/**
 * @brief 最近一帧腿反馈：四个关节的位置和速度，外加"这一帧里有没有它"
 */
struct LegState
{
    std::array<double, 4> position_rad   {};   // rad
    std::array<double, 4> velocity_radps {};   // rad/s
    std::array<bool,   4> valid          {};   // false 时上面两个值不可信
};

/**
 * @brief 腿姿保持器：四个腿关节各一条 PD，外加静态重力前馈
 *
 * @note 控制律是逐关节的：tau = sign·(U0 + kp·(目标 - q) - kd·q̇)，再夹到 max_effort。
 *       U0 按每条腿**实测**的角度现算（不是常数），腿歪到哪就补到哪
 */
class LegNode : public rclcpp::Node
{
public:
    // 下标顺序固定 [左髋, 右髋, 左膝, 右膝]，每个数组都按它对齐，别再散着写
    static constexpr std::size_t kLeftHip    = 0;
    static constexpr std::size_t kRightHip   = 1;
    static constexpr std::size_t kLeftKnee   = 2;
    static constexpr std::size_t kRightKnee  = 3;
    static constexpr std::size_t kJointCount = 4;

    // 每条腿的 (胯下标, 膝下标) —— 重力前馈要的是同一条腿的两个角
    static constexpr std::array<std::pair<std::size_t, std::size_t>, 2> kLegJoints{{{kLeftHip, kLeftKnee}, {kRightHip, kRightKnee}}};

    LegNode() : rclcpp::Node("leg")
    {
        // ---- 腿几何与质量：算 U0 用，见 modules/leg.xacro ----
        geometry_.leg_length = declare_parameter("leg_length", 0.056);
        geometry_.leg_angle  = declare_parameter("leg_angle",  0.6981317008);
        geometry_.body_mass  = declare_parameter("body_mass",  1.25);
        geometry_.rod_mass   = declare_parameter("rod_mass",   0.03);
        geometry_.joint_mass = declare_parameter("joint_mass", 0.005);
        geometry_.wheel_mass = declare_parameter("wheel_mass", 0.15);
        geometry_.gravity    = declare_parameter("gravity",    9.81);

        // ---- 保持参数 ----
        hold_.dt                 = declare_parameter("dt",                 0.001);
        hold_.target_hip_rad     = declare_parameter("target_hip_rad",     0.0);
        hold_.target_knee_rad    = declare_parameter("target_knee_rad",    0.0);
        hold_.kp                 = std::abs(declare_parameter("kp",                     2.0));
        hold_.kd                 = std::abs(declare_parameter("kd",                     0.05));
        hold_.max_effort         = std::abs(declare_parameter("max_effort",             1.0));
        hold_.feedback_timeout_s = std::abs(declare_parameter("feedback_timeout_s",     0.10));
        hold_.joint_sign         = declare_parameter<std::vector<double>>("joint_sign", {1.0, 1.0, 1.0, 1.0});

        if (geometry_.leg_length <= 0.0 || geometry_.body_mass <= 0.0 || hold_.dt <= 0.0)
        {
            throw std::invalid_argument("leg_length / body_mass / dt 必须大于 0");
        }
        if (hold_.joint_sign.size() != kJointCount)
        {
            throw std::invalid_argument("joint_sign 要 4 个数，顺序 [左髋, 右髋, 左膝, 右膝]");
        }

        // ---- 接线：四个关节叫什么、指令发到哪，顺序同上。跟 /chassis 段的 leg_joint_names /
        //      leg_command_topics 是一份东西，改一处要跟着改另一处 ----
        joint_names_ = declare_parameter<std::vector<std::string>>("joint_names",
            {"left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"});

        command_topics_ = declare_parameter<std::vector<std::string>>("command_topics",
            {"/motor/left_hip/command", "/motor/right_hip/command", "/motor/left_knee/command", "/motor/right_knee/command"});

        if (joint_names_.size() != kJointCount || command_topics_.size() != kJointCount)
        {
            throw std::invalid_argument("joint_names / command_topics 各要 4 条，顺序 [左髋, 右髋, 左膝, 右膝]");
        }

        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            command_pub_[i] = create_publisher<std_msgs::msg::Float64>(command_topics_[i], 10);
        }

        const std::string joint_states_topic = declare_parameter("joint_states_topic", "/joint_states");

        // 回调只缓存最新一帧，控制律统一在定时器那一拍算：跟话题频率解耦，也不会多算
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(*msg); });

        timer_ = create_wall_timer(std::chrono::duration<double>(hold_.dt), [this]() { update(); });

        // 开跑前把标准姿态下的 U0 打出来：这两个数是量级和方向的第一个检查点，
        // 手算/看日志对不上说明质量或几何抄错了（见 gravity_effort 的注释）
        const auto [u0_hip, u0_knee] = gravity_effort(hold_.target_hip_rad, hold_.target_knee_rad);
        RCLCPP_INFO(get_logger(), "标准姿态 U0：胯 %.4f N·m，膝 %.4f N·m", u0_hip, u0_knee);
    }

private:
    LegGeometry geometry_;
    HoldParams  hold_;

    // 接线：四个关节名和四条指令话题，顺序 [左髋, 右髋, 左膝, 右膝]
    std::vector<std::string> joint_names_;
    std::vector<std::string> command_topics_;

    // 发给四个腿电机的指令（力矩 N·m）。电机在自己的进程里订这四条话题
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kJointCount> command_pub_{};

    LegState state_;    // 最近一帧腿反馈

    bool                                   has_joint_state_ {false};   // 收到过"四个关节都在"的一帧没有
    std::chrono::steady_clock::time_point  last_feedback_time_{};      // 用 steady_clock：仿真时间会被暂停/复位

    // ---- ROS 句柄 ----
    rclcpp::TimerBase::SharedPtr                                  timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_state_sub_;

    /**
     * @brief 绕 +y 转 q 之后，局部点 (x, z) 相对关节的横向偏移（x 向前、z 向上）
     *
     * @note 只有 x 要算：竖直力的力矩只跟横向偏移有关，z 不进来
     */
    static double rotate_x(double q, double x, double z)
    {
        return x * std::cos(q) + z * std::sin(q);
    }

    /**
     * @brief 竖直力 fz 在横向偏移 dx 处，对关节产生的绕 +y 力矩
     */
    static double moment_about_y(double dx, double fz)
    {
        return -dx * fz;
    }

    /**
     * @brief 静态重力前馈 U0：这个姿态下关节**必须**自己出的力矩（N·m），拿它顶住自重
     *
     * @note 模型：整车重量两条腿平分（质心在两条腿正中间，单腿踩台阶也先这么算），地面法向力
     *       竖直向上、作用在轮心正下方。关节以外的每个竖直力对关节的力矩求和，反号就是关节
     *       该出的力矩 —— 大腿的远端在 (span, -drop)、小腿的远端再往回 (−span, −drop)，标准
     *       姿态下轮心正好落在胯轴正下方
     * @note 标准姿态下算出来 ≈ 胯 -0.015、膝 -0.257 N·m（负 = 往"把腿顶直"那边出）。膝那份
     *       大是因为整车重量在两腿上的分量对膝的力臂是整个 span；胯那份只剩连杆自重
     */
    std::pair<double, double> gravity_effort(double hip, double knee) const
    {
        const double span      = geometry_.leg_length * std::cos(geometry_.leg_angle);
        const double drop      = geometry_.leg_length * std::sin(geometry_.leg_angle);
        const double half_span = 0.5 * span;
        const double half_drop = 0.5 * drop;

        const double knee_x      = rotate_x(hip, span, -drop);
        const double wheel_x     = knee_x + rotate_x(hip + knee, -span, -drop);
        const double thigh_com_x = rotate_x(hip, half_span, -half_drop);
        const double calf_com_x  = knee_x + rotate_x(hip + knee, -half_span, -half_drop);

        // 地面对这条腿的法向力：整车重量（车身 + 两个轮子）两腿平分
        const double normal_force = 0.5 * (geometry_.body_mass + 2.0 * geometry_.wheel_mass) * geometry_.gravity;

        const double rod_weight   = -geometry_.rod_mass * geometry_.gravity;
        const double wheel_weight = -geometry_.wheel_mass * geometry_.gravity;
        const double joint_weight = -geometry_.joint_mass * geometry_.gravity;

        // 膝：远端是小腿 + 轮子 + 地面法向力（大腿和膝上圆柱在膝这边算近端，不算）
        const double knee_external = moment_about_y(wheel_x - knee_x, normal_force)
                                   + moment_about_y(calf_com_x - knee_x, rod_weight)
                                   + moment_about_y(wheel_x - knee_x, wheel_weight);

        // 胯：整条腿都在远端（大腿、膝上圆柱、小腿、轮子）+ 地面法向力
        const double hip_external  = moment_about_y(wheel_x, normal_force)
                                   + moment_about_y(thigh_com_x, rod_weight)
                                   + moment_about_y(knee_x, joint_weight)
                                   + moment_about_y(calf_com_x, rod_weight)
                                   + moment_about_y(wheel_x, wheel_weight);

        return {-hip_external, -knee_external};
    }

    /**
     * @brief 收到一帧关节状态：挑出四个腿关节，四个都在才算数
     *
     * @note name / position / velocity 三个数组不保证一样长，一起截短，别各自判越界
     */
    void on_joint_state(const sensor_msgs::msg::JointState& msg)
    {
        const std::size_t count = std::min({msg.name.size(), msg.position.size(), msg.velocity.size()});

        // 先全置无效，找到哪个再置回来：宁可这拍当没有，也别拿上一帧的旧角度当现值
        state_.valid.fill(false);

        for (std::size_t i = 0; i < count; ++i)
        {
            for (std::size_t joint = 0; joint < kJointCount; ++joint)
            {
                if (msg.name[i] == joint_names_[joint])
                {
                    state_.position_rad[joint]   = msg.position[i];
                    state_.velocity_radps[joint] = msg.velocity[i];
                    state_.valid[joint]          = true;
                    break;
                }
            }
        }

        has_joint_state_    = std::all_of(state_.valid.begin(), state_.valid.end(), [](bool ok) { return ok; });
        last_feedback_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 最后一帧反馈到现在过了多久（一帧都没收到时返回无穷大）
     */
    double feedback_age_s() const
    {
        if (!has_joint_state_)
        {
            return std::numeric_limits<double>::infinity();
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_feedback_time_).count();
    }

    /**
     * @brief 单个关节的出力：前馈 + PD，按符号翻向后夹到上限（N·m）
     *
     * @note 符号是整体翻的：joint_sign = -1 时前馈和 PD 一起翻 —— 正方向反了就是这样
     */
    double hold_effort(std::size_t joint, double feedforward, double target) const
    {
        const double effort = feedforward
                            + hold_.kp * (target - state_.position_rad[joint])
                            - hold_.kd * state_.velocity_radps[joint];
        return std::clamp(hold_.joint_sign[joint] * effort, -hold_.max_effort, hold_.max_effort);
    }

    /**
     * @brief 控制节拍：按最新一帧腿角算 U0 和 PD，把四个关节的力矩发出去
     */
    void update()
    {
        // 反馈没来、缺关节或者已经超时就不发力：拿 0 当状态等于盲发力
        if (!has_joint_state_ || feedback_age_s() > hold_.feedback_timeout_s)
        {
            publish_effort({0.0, 0.0, 0.0, 0.0});
            return;
        }

        std::array<double, kJointCount> effort{};

        // 每条腿各算各的：U0 只跟本条腿的姿态有关，两条腿的姿态可以不一样
        for (const auto& leg : kLegJoints)
        {
            const auto [hip, knee] = leg;
            const auto [u0_hip, u0_knee] = gravity_effort(state_.position_rad[hip], state_.position_rad[knee]);

            effort[hip]  = hold_effort(hip,  u0_hip,  hold_.target_hip_rad);
            effort[knee] = hold_effort(knee, u0_knee, hold_.target_knee_rad);
        }

        publish_effort(effort);

        // 腿姿诊断：阶段 2 验收看的就是"四个角有没有稳在 0 附近、力矩是不是这个量级"。
        // 顺序 [左髋, 右髋, 左膝, 右膝]，节流到 1 s 一条
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "腿姿 [左髋 右髋 左膝 右膝] q = %.4f %.4f %.4f %.4f rad，力矩 = %.3f %.3f %.3f %.3f N·m",
                             state_.position_rad[kLeftHip], state_.position_rad[kRightHip],
                             state_.position_rad[kLeftKnee], state_.position_rad[kRightKnee],
                             effort[kLeftHip], effort[kRightHip], effort[kLeftKnee], effort[kRightKnee]);
    }

    /**
     * @brief 把四个关节的力矩发出去（N·m）
     */
    void publish_effort(const std::array<double, kJointCount>& effort)
    {
        std_msgs::msg::Float64 command;

        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            command.data = effort[i];
            command_pub_[i]->publish(command);
        }
    }
};
