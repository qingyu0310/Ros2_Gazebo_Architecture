/**
 * @file chassis.hpp
 * @author qingyu
 * @brief 底盘节点：轮腿车的 8 状态 3 输入全身平衡控制器
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 * @note 分工：求解器在 framework，单轮的限幅/速度环在 ros2_layer 的 MotorNode 里。
 *       本节点只把状态喂给 LQR，再把力矩按偏航差分分给两个轮子；电机在独立进程 node/motor
 * @note 话题：订 /imu、/joint_states、/chassis/cmd_vel；发六路 /motor/<执行器>/command（N·m）
 * @note K/X0/U0 从离线生成的 gains/wheel_leg/nominal.yaml 加载；接口或模型不一致时拒绝启动
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <yaml-cpp/yaml.h>

#include "leg_height.hpp"
#include "torque_allocator.hpp"

/**
 * @brief 模型参数：这台车的物理量，由 URDF 量出来
 */
struct ModelParams
{
    double wheel_radius       {0.030};                                                  // r，跟 modules/wheel.xacro 的 radius 对齐
    double cart_mass          {0.9000};                                                 // M，两轮质量 + 各自自转惯量/r² 的折合
    double body_mass          {1.25};                                                   // m，除轮子外的车身质量
    double body_com_height    {0.06825};                                                // l，车身质心到轮轴的距离
    double body_pitch_inertia {0.002591};                                               // I，车身绕质心、绕 y 轴的俯仰惯量
    double cart_damping       {0.02};                                                   // b，等效粘性阻尼，估的量
    double gravity            {9.81};
};

/**
 * @brief 控制参数：离散化步长、LQR 权重、限幅与符号
 */
struct ControlParams
{
    double              dt             {0.001};                                         // 控制周期，要跟物理步长和定时器实际周期一致
    double              max_effort     {0.25};                                          // 公共力矩上限 N·m（附着极限 0.30）
    double              fall_angle_rad {0.70};                                          // |俯仰| 超过它就认定倒地，力矩归零
    double              pitch_sign     {1.0};
    double              control_sign   {1.0};
    double              wheel_effort_rate_limit {100.0};                                // 单轮力矩变化率 N·m/s
    std::string         mode           {"effort"};                                   // 只能是 effort，速度环会和平衡环抢

    std::vector<double> q_roll         {800.0, 60.0, 400.0, 40.0, 400.0, 40.0};   // [roll, φ̇, q_h-, q̇_h-, q_k-, q̇_k-]
    std::vector<double> r_roll         {2.0, 2.0};                                                // [tau_h-, tau_k-]
};

/**
 * @brief 髋膝参数：四个腿关节，顺序固定 [左髋, 右髋, 左膝, 右膝]，每个数组都按这个顺序对齐
 *
 * @note 上下限抄的是模型里的硬限位（髋 -15°~35°、膝 -65°~25°）。控制器要用的软限位得往里收，
 *       留出恢复力矩的余量；力矩/速度上限见 lqr 那台的 leg_effort 1.5、leg_velocity 6.0
 */
struct LegParams
{
    std::vector<std::string> joint_names       {"left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"};
    std::vector<std::string> command_topics    {"/motor/left_hip/command", "/motor/right_hip/command", "/motor/left_knee/command", "/motor/right_knee/command"};
    std::vector<double>      max_effort        {1.5, 1.5, 1.5, 1.5};                    // 单关节力矩上限 N·m
    std::vector<double>      max_speed_radps   {6.0, 6.0, 6.0, 6.0};                    // 关节速度上限 rad/s
    std::vector<double>      limit_lower       {-0.2618, -0.2618, -1.1345, -1.1345};    // 下限 rad
    std::vector<double>      limit_upper       {0.6109, 0.6109, 0.4363, 0.4363};        // 上限 rad
    std::vector<double>      joint_sign        {1.0, 1.0, 1.0, 1.0};                    // 正方向反了翻成 -1.0
    std::vector<double>      effort_rate_limit {500.0, 500.0, 500.0, 500.0};            // 力矩变化率 N·m/s
    std::vector<double>      soft_limit_margin {0.10, 0.10, 0.10, 0.10};                // 软限位相对硬限位向内收缩 rad
    std::vector<double>      soft_limit_kp     {2.0, 2.0, 2.0, 2.0};                    // 越过软限位后的恢复刚度 N·m/rad
    std::vector<double>      soft_limit_kd     {0.05, 0.05, 0.05, 0.05};                // 软限位恢复阻尼 N·m·s/rad
};

struct SafetyParams
{
    double feedback_timeout_s   {0.10};    // IMU 或关节反馈超过此时间未更新就安全停机
    double max_control_period_s {0.02};    // 控制回调最大允许间隔
    double recovery_dwell_s     {0.20};    // 故障消失后持续健康这么久才重新接管
};

struct ActuatorEnableParams
{
    bool wheel_common {true};
    bool hip_common {true};
    bool knee_common {true};
    bool hip_diff {true};
    bool knee_diff {true};
};

struct HeightParams
{
    double link_length      {0.056};            // 单根连杆长度 m
    double link_angle       {0.6981317008};     // 零位时连杆与水平面的夹角 rad
    double target_height_m  {0.0719922123};     // 髋轴到轮轴的目标竖直距离 m
    double maximum_rate_mps {0.01};             // 高度目标最大变化速度 m/s
    double rod_mass         {0.03};
    double joint_mass       {0.005};
    double wheel_mass       {0.15};
};

/**
 * @brief 底盘状态：最近一帧反馈里挑出来的量，前四个就是 LQR 的 x
 *
 * @note 取的是两个轮子的**平均**：差的那一半是偏航，不算进里程
 */
struct ChassisState
{
    double roll            {0.0};       // phi，车身横滚角，左侧抬起为正
    double roll_rate       {0.0};       // phi_dot
    double pitch           {0.0};       // θ，车身俯仰角，前倾为正
    double pitch_rate      {0.0};       // θ̇
    double yaw_rate        {0.0};       // 偏航角速度，只给转向环用
    double wheel_position  {0.0};       // r·(pos_L+pos_R)/2，轮子转过的角折成"走了多远"
    double wheel_velocity  {0.0};       // ṗ
    double position_origin {0.0};       // 位移原点：减掉它才是里程

    // 四个腿关节，下标跟 LegParams::joint_names 对齐（[左髋, 右髋, 左膝, 右膝]）。
    // 现在还没人用（全身控制器是文档 §8 阶段 3），先把反馈收进来，链路对不对看得见
    std::array<double, 4> leg_position_rad {};
    std::array<double, 4> leg_velocity_radps {};
    std::array<bool,   4> leg_valid {};  // 这一帧里有没有它；false 时上面两个值不可信
};

/**
 * @brief 遥控目标：cmd_vel 解出来的两个目标值，外加一个由速度积分出来的位置目标
 *
 * @note position 由速度积分而来：LQR 追的是速度，它只给位移项一个参照
 */
struct TeleopTarget
{
    double velocity {0.0};              // 目标前进速度 m/s，来自 cmd_vel 的 linear.x
    double yaw_rate {0.0};              // 目标偏航角速度 rad/s，来自 cmd_vel 的 angular.z
    double position {0.0};              // 位置目标，由 velocity 积分而来
};

/**
 * @brief 标志位：反馈来没来、原点标定没标定、上一拍是不是判了倒地
 *
 * @note 反馈标志必须先置上才发力：拿 0 当状态等于盲发力（见 update()）
 */
struct StatusFlags
{
    bool has_imu            {false};    // 收到过 IMU 没有
    bool has_joint_state    {false};    // 收到过"两个轮子都在"的一帧没有
    bool origin_initialized {false};    // 位移原点标定过没有
    bool has_command        {false};    // 收到过 cmd_vel 没有
};

/**
 * @brief 遥控指令接口：linear.x 前后方向（+1=前进满速），angular.z 偏航方向（+1=左转满速）
 *
 * @note 进来的量是**归一化**的：±1 = 满速，实际多快由 max_velocity / max_yaw_rate 定
 */
struct RemoteParams
{
    std::string cmd_vel_topic     {"/chassis/cmd_vel"};
    double      max_velocity      {0.20};
    double      max_yaw_rate      {1.0};
    double      height_step_m     {0.001};                                              // 每个键盘身高脉冲改变的目标高度 m
    double      yaw_kp            {0.01};                                               // 偏航误差 -> 力矩差 N·m·s/rad
    double      max_differential  {0.02};                                               // 力矩差上限，单轮上限 = 它 + max_effort
    double      command_timeout_s {0.30};                                               // 这么久没收到 cmd_vel 就当停车
};

/**
 * @brief 反馈话题：要跟模型里两个传感器宏的话题对上
 */
struct TopicParams
{
    std::string imu_topic          {"/imu"};
    std::string joint_states_topic {"/joint_states"};
};

/**
 * @brief 轮腿车底盘：全身俯仰共模 LQR + 轮子偏航差动
 *
 * @note U=[轮共模,髋共模,膝共模]，一个共模量就是左右每个执行器各自要出的力矩
 */
class ChassisNode : public rclcpp::Node
{
public:
    static constexpr std::size_t kStateDim = 8;
    static constexpr std::size_t kInputDim = 3;
    using State      = Eigen::Matrix<double, kStateDim, 1>;
    using Control    = Eigen::Matrix<double, kInputDim, 1>;
    using GainMatrix = Eigen::Matrix<double, kInputDim, kStateDim>;

    static constexpr std::size_t kRollStateDim = 4;
    static constexpr std::size_t kRollInputDim = 2;
    using RollState      = Eigen::Matrix<double, kRollStateDim, 1>;
    using RollControl    = Eigen::Matrix<double, kRollInputDim, 1>;
    using RollStateMatrix = Eigen::Matrix<double, kRollStateDim, kRollStateDim>;
    using RollInputMatrix = Eigen::Matrix<double, kRollStateDim, kRollInputDim>;
    using RollGainMatrix  = Eigen::Matrix<double, kRollInputDim, kRollStateDim>;

    struct GainPoint
    {
        double height {0.0};
        GainMatrix gain {GainMatrix::Zero()};
        State x0 {State::Zero()};
        Control u0 {Control::Zero()};
        RollGainMatrix roll_gain {RollGainMatrix::Zero()};
        RollState roll_x0 {RollState::Zero()};
        RollControl roll_u0 {RollControl::Zero()};
        Eigen::Matrix<double, 1, 2> roll_per_q {Eigen::Matrix<double, 1, 2>::Zero()};
    };

    // 轮子下标。数组全按这个顺序对齐，别再散着写
    static constexpr std::size_t kLeft       = 0;
    static constexpr std::size_t kRight      = 1;
    static constexpr std::size_t kWheelCount = 2;

    ChassisNode() : rclcpp::Node("chassis")
    {
        // ---- 模型参数：由 URDF 量出来，见 docs/轮腿机器人LQR平衡教程.md §1.3 ----
        model_.wheel_radius       = declare_parameter("wheel_radius",       0.030);
        model_.cart_mass          = declare_parameter("cart_mass",          0.9000);
        model_.body_mass          = declare_parameter("body_mass",          1.25);
        model_.body_com_height    = declare_parameter("body_com_height",    0.06825);
        model_.body_pitch_inertia = declare_parameter("body_pitch_inertia", 0.002591);
        model_.cart_damping       = declare_parameter("cart_damping",       0.02);
        model_.gravity            = declare_parameter("gravity",            9.81);

        // ---- 控制 ----
        control_.dt             = declare_parameter("dt",                         0.001);
        control_.max_effort     = std::abs(declare_parameter("max_effort",     0.25));
        control_.fall_angle_rad = std::abs(declare_parameter("fall_angle_rad", 0.70));
        control_.pitch_sign     = declare_parameter("pitch_sign",                 1.0);
        control_.control_sign   = declare_parameter("control_sign",               1.0);
        control_.wheel_effort_rate_limit = std::abs(declare_parameter("wheel_effort_rate_limit", 100.0));
        control_.mode           = declare_parameter("control_mode",               "effort");

        // ---- Q/R 只给离线工具用；运行时 Pitch/Roll 的 K 都从增益文件读 ----
        control_.q_roll  = declare_parameter<std::vector<double>>("q_roll",  {800.0, 60.0, 400.0, 40.0, 400.0, 40.0});
        control_.r_roll  = declare_parameter<std::vector<double>>("r_roll",  {2.0, 2.0});

        // 平衡环要的是"给我这个力矩"，插一层速度环就是多一个积分器 + 一层延迟，两个环互相顶
        if (control_.mode != "effort")
        {
            throw std::invalid_argument("平衡环必须用 effort 模式：control_mode 只能是 'effort'");
        }

        // ---- 遥控接口 ----
        remote_.cmd_vel_topic     = declare_parameter("cmd_vel_topic",                 "/chassis/cmd_vel");
        remote_.max_velocity      = std::abs(declare_parameter("max_velocity",      0.20));
        remote_.max_yaw_rate      = std::abs(declare_parameter("max_yaw_rate",      1.0));
        remote_.height_step_m     = declare_parameter("height_step_m",                 0.001);
        remote_.yaw_kp            = std::abs(declare_parameter("yaw_kp",            0.01));
        remote_.max_differential  = std::abs(declare_parameter("max_differential",  0.02));
        remote_.command_timeout_s = std::abs(declare_parameter("command_timeout_s", 0.30));

        safety_.feedback_timeout_s   = std::abs(declare_parameter("feedback_timeout_s", 0.10));
        safety_.max_control_period_s = std::abs(declare_parameter("max_control_period_s", 0.02));
        safety_.recovery_dwell_s     = std::abs(declare_parameter("recovery_dwell_s", 0.02));

        enabled_.wheel_common      = declare_parameter("enable_wheel_common", true);
        enabled_.hip_common        = declare_parameter("enable_hip_common",   true);
        enabled_.knee_common       = declare_parameter("enable_knee_common",  true);
        enabled_.hip_diff          = declare_parameter("enable_hip_diff",     true);
        enabled_.knee_diff         = declare_parameter("enable_knee_diff",    true);

        // ---- 反馈话题 ----
        topics_.imu_topic          = declare_parameter("imu_topic",           "/imu");
        topics_.joint_states_topic = declare_parameter("joint_states_topic",  "/joint_states");

        const std::string gain_directory =
            ament_index_cpp::get_package_share_directory("project") + "/gains/wheel_leg/";
        const std::string default_gain_file = gain_directory + "nominal.yaml";
        gain_file_ = declare_parameter("gain_file", default_gain_file);
        const std::vector<std::string> default_gain_files{
            "height_058.yaml", "nominal.yaml", "height_080.yaml",
            "height_090.yaml", "height_100.yaml", "height_104.yaml"};
        gain_files_ = declare_parameter<std::vector<std::string>>("gain_files", default_gain_files);
        const std::string robot_description = declare_parameter("robot_description", std::string{});
        if (gain_files_.empty())
        {
            gain_files_.push_back(gain_file_);
        }
        for (const auto& configured_path : gain_files_)
        {
            const std::string path = configured_path.find('/') == std::string::npos
                ? gain_directory + configured_path : configured_path;
            load_gain(path, robot_description);
            gain_schedule_.push_back({loaded_gain_height_, gain_, x0_, u0_, roll_gain_,
                                      roll_x0_, roll_u0_, roll_per_q_});
        }
        std::sort(gain_schedule_.begin(), gain_schedule_.end(),
                  [](const GainPoint& left, const GainPoint& right) { return left.height < right.height; });
        for (std::size_t i = 1; i < gain_schedule_.size(); ++i)
        {
            if (gain_schedule_[i].height - gain_schedule_[i - 1].height < 1e-9)
            {
                throw std::runtime_error("增益调度表存在重复高度工作点");
            }
        }

        // ---- 两个轮子的接线：哪个关节、指令发到哪个话题。都按下标跟 kLeft / kRight 对齐 ----
        joint_names_    = {declare_parameter("left_joint_name",     "left_wheel_joint"),
                           declare_parameter("right_joint_name",    "right_wheel_joint")};

        command_topics_ = {declare_parameter("left_command_topic",  "/motor/left/command"),
                           declare_parameter("right_command_topic", "/motor/right/command")};

        for (std::size_t i = 0; i < kWheelCount; ++i)
        {
            command_pub_[i] = create_publisher<std_msgs::msg::Float64>(command_topics_[i], 10);
        }

        // ---- 髋膝：四个腿关节的接线和限幅，顺序 [左髋, 右髋, 左膝, 右膝]，每个数组都按它对 ----
        leg_.joint_names = declare_parameter<std::vector<std::string>>("leg_joint_names",
            {"left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"});

        leg_.command_topics = declare_parameter<std::vector<std::string>>("leg_command_topics",
            {"/motor/left_hip/command",     "/motor/right_hip/command", "/motor/left_knee/command",    "/motor/right_knee/command"});

        leg_.max_effort      = declare_parameter<std::vector<double>>("leg_max_effort",      {1.5, 1.5, 1.5, 1.5});
        leg_.max_speed_radps = declare_parameter<std::vector<double>>("leg_max_speed_radps", {6.0, 6.0, 6.0, 6.0});
        leg_.limit_lower     = declare_parameter<std::vector<double>>("leg_limit_lower",     {-0.2618, -0.2618, -1.1345, -1.1345});
        leg_.limit_upper     = declare_parameter<std::vector<double>>("leg_limit_upper",     {0.6109, 0.6109, 0.4363, 0.4363});
        leg_.joint_sign      = declare_parameter<std::vector<double>>("leg_joint_sign",      {1.0, 1.0, 1.0, 1.0});
        leg_.effort_rate_limit = declare_parameter<std::vector<double>>("leg_effort_rate_limit", {500.0, 500.0, 500.0, 500.0});
        leg_.soft_limit_margin = declare_parameter<std::vector<double>>("leg_soft_limit_margin", {0.10, 0.10, 0.10, 0.10});
        leg_.soft_limit_kp = declare_parameter<std::vector<double>>("leg_soft_limit_kp", {2.0, 2.0, 2.0, 2.0});
        leg_.soft_limit_kd = declare_parameter<std::vector<double>>("leg_soft_limit_kd", {0.05, 0.05, 0.05, 0.05});

        height_.link_length      = declare_parameter("leg_length",      0.056);
        height_.link_angle       = declare_parameter("leg_angle",       0.6981317008);
        height_.target_height_m  = declare_parameter("target_height_m", 0.0719922123);
        height_.maximum_rate_mps = declare_parameter("height_rate_mps", 0.01);
        height_.rod_mass         = declare_parameter("leg_rod_mass",    0.03);
        height_.joint_mass       = declare_parameter("leg_joint_mass",  0.005);
        height_.wheel_mass       = declare_parameter("wheel_mass",      0.15);

        validate_leg_params();
        configure_allocator();
        configure_height_kinematics();
        for (std::size_t i = 0; i < leg_.command_topics.size(); ++i)
        {
            leg_command_pub_[i] = create_publisher<std_msgs::msg::Float64>(leg_.command_topics[i], 10);
        }

        // 回调只缓存最新一帧，控制律统一在定时器那一拍算：跟话题频率解耦，也不会多算
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            topics_.imu_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            topics_.joint_states_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(*msg); });

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            remote_.cmd_vel_topic, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(*msg); });

        parameter_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) { return on_parameters(parameters); });

        timer_ = create_wall_timer(std::chrono::duration<double>(control_.dt), [this]() { update(); });

        RCLCPP_INFO(get_logger(), "腿高范围 [%.4f, %.4f] m，目标 %.4f m，变化率 %.4f m/s",
                    height_kinematics_->minimum_height(), height_kinematics_->maximum_height(),
                    height_.target_height_m, height_.maximum_rate_mps);
    }

private:
    enum class SafetyState
    {
        kWaitingFeedback,
        kRecovering,
        kActive,
        kFeedbackTimeout,
        kInvalidState,
        kFallen,
        kJointLimit,
        kTimingFault
    };

    GainMatrix  gain_ {GainMatrix::Zero()};
    State       x0_   {State::Zero()};
    Control     u0_   {Control::Zero()};
    RollStateMatrix roll_a_    {RollStateMatrix::Zero()};
    RollInputMatrix roll_b_    {RollInputMatrix::Zero()};
    RollGainMatrix  roll_gain_ {RollGainMatrix::Zero()};
    RollState       roll_x0_   {RollState::Zero()};
    RollControl     roll_u0_   {RollControl::Zero()};
    Eigen::Matrix<double, 1, 2> roll_per_q_ {Eigen::Matrix<double, 1, 2>::Zero()};
    std::string gain_file_;
    std::vector<std::string> gain_files_;
    std::vector<GainPoint> gain_schedule_;
    double loaded_gain_height_ {0.0};

    ModelParams   model_;
    ControlParams control_;
    RemoteParams  remote_;
    TopicParams   topics_;
    LegParams     leg_;
    SafetyParams  safety_;
    ActuatorEnableParams enabled_;
    HeightParams height_;
    std::unique_ptr<TorqueAllocator> allocator_;
    std::unique_ptr<LegHeightKinematics> height_kinematics_;
    LegHeightKinematics::Reference height_reference_ {};

    // 接线：哪个关节、指令发到哪个话题。两组都按下标对齐
    std::array<std::string, kWheelCount> joint_names_{};
    std::array<std::string, kWheelCount> command_topics_{};

    // 发给两个电机的指令（力矩 N·m）。电机在自己的进程里订这两个话题
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kWheelCount> command_pub_{};
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> leg_command_pub_{};

    ChassisState state_;    // 最近一帧反馈，加上控制用的那四个量
    TeleopTarget teleop_;   // 遥控目标值
    StatusFlags  flags_;    // 反馈/标定/倒地，都是标志位

    // 最后一条 cmd_vel 的时刻。用 steady_clock：仿真时间会被暂停/复位
    std::chrono::steady_clock::time_point last_command_time_{};
    std::chrono::steady_clock::time_point last_imu_time_{};
    std::chrono::steady_clock::time_point last_joint_state_time_{};
    std::chrono::steady_clock::time_point last_update_time_{};
    std::chrono::steady_clock::time_point healthy_since_{};
    bool has_update_time_ {false};
    bool has_healthy_since_ {false};
    bool ever_active_ {false};
    SafetyState safety_state_ {SafetyState::kWaitingFeedback};

    // ---- ROS 句柄 ----
    rclcpp::TimerBase::SharedPtr                                   timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

    rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters)
        {
            const std::string& name = parameter.get_name();
            try
            {
                if (name == "enable_wheel_common") enabled_.wheel_common = parameter.as_bool();
                else if (name == "enable_hip_common") enabled_.hip_common = parameter.as_bool();
                else if (name == "enable_knee_common") enabled_.knee_common = parameter.as_bool();
                else if (name == "enable_hip_diff") enabled_.hip_diff = parameter.as_bool();
                else if (name == "enable_knee_diff") enabled_.knee_diff = parameter.as_bool();
                else if (name == "target_height_m")
                {
                    const double target = parameter.as_double();
                    if (!std::isfinite(target) || target < height_kinematics_->minimum_height() ||
                        target > height_kinematics_->maximum_height())
                    {
                        result.successful = false;
                        result.reason = "target_height_m 超出软限位可达范围 [" +
                            std::to_string(height_kinematics_->minimum_height()) + ", " +
                            std::to_string(height_kinematics_->maximum_height()) + "]";
                        return result;
                    }
                    height_.target_height_m = target;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 250, "新腿高目标 %.4f m", target);
                    continue;
                }
                else if (name == "height_rate_mps")
                {
                    const double rate = parameter.as_double();
                    if (!std::isfinite(rate) || rate <= 0.0)
                    {
                        result.successful = false;
                        result.reason = "height_rate_mps 必须是有限正数";
                        return result;
                    }
                    height_.maximum_rate_mps = rate;
                    RCLCPP_INFO(get_logger(), "腿高变化率改为 %.4f m/s", rate);
                    continue;
                }
                else continue;
                RCLCPP_WARN(get_logger(), "执行器消融开关变更：%s=%s", name.c_str(), parameter.as_bool() ? "true" : "false");
            }
            catch (const rclcpp::ParameterTypeException& error)
            {
                result.successful = false;
                result.reason = error.what();
                return result;
            }
        }
        return result;
    }

    static std::uint64_t fnv1a(const std::string& text)
    {
        std::uint64_t hash = 1469598103934665603ULL;
        for (const unsigned char c : text)
        {
            hash ^= c;
            hash *= 1099511628211ULL;
        }
        return hash;
    }

    static std::string model_hash_text(const std::string& xml)
    {
        std::string uncommented;
        uncommented.reserve(xml.size());
        std::size_t cursor = 0;
        while (cursor < xml.size())
        {
            const std::size_t comment = xml.find("<!--", cursor);
            if (comment == std::string::npos)
            {
                uncommented.append(xml, cursor, std::string::npos);
                break;
            }
            uncommented.append(xml, cursor, comment - cursor);
            const std::size_t end = xml.find("-->", comment + 4);
            if (end == std::string::npos)
            {
                throw std::runtime_error("robot_description 里的 XML 注释没有闭合");
            }
            cursor = end + 3;
        }
        std::string normalized;
        normalized.reserve(uncommented.size());
        char quote = '\0';
        for (const unsigned char c : uncommented)
        {
            if (quote == '\0' && (c == '\'' || c == '"'))
            {
                quote = static_cast<char>(c);
                normalized.push_back(static_cast<char>(c));
            }
            else if (quote != '\0' && c == static_cast<unsigned char>(quote))
            {
                quote = '\0';
                normalized.push_back(static_cast<char>(c));
            }
            else if (quote != '\0' || !std::isspace(c))
            {
                normalized.push_back(static_cast<char>(c));
            }
        }
        return normalized;
    }

    static std::string hash_string(const std::string& text)
    {
        std::ostringstream out;
        out << "0x" << std::hex << fnv1a(model_hash_text(text));
        return out.str();
    }

    template<std::size_t Size>
    static void require_layout(const YAML::Node& node, const std::array<const char*, Size>& expected,
                               const std::string& name)
    {
        if (!node.IsSequence() || node.size() != Size)
        {
            throw std::runtime_error(name + " 维度不是 " + std::to_string(Size));
        }
        for (std::size_t i = 0; i < Size; ++i)
        {
            if (node[i].as<std::string>() != expected[i])
            {
                throw std::runtime_error(name + " 第 " + std::to_string(i) + " 项不匹配：期望 " + expected[i]);
            }
        }
    }

    template<int Rows, int Columns>
    static Eigen::Matrix<double, Rows, Columns> load_matrix(const YAML::Node& node, const std::string& name)
    {
        if (!node.IsSequence() || node.size() != static_cast<std::size_t>(Rows))
        {
            throw std::runtime_error(name + " 行数不是 " + std::to_string(Rows));
        }

        Eigen::Matrix<double, Rows, Columns> result;
        for (int row = 0; row < Rows; ++row)
        {
            if (!node[row].IsSequence() || node[row].size() != static_cast<std::size_t>(Columns))
            {
                throw std::runtime_error(name + " 第 " + std::to_string(row) + " 行列数不是 " + std::to_string(Columns));
            }
            for (int column = 0; column < Columns; ++column)
            {
                result(row, column) = node[row][column].as<double>();
            }
        }
        return result;
    }

    /** @brief 加载离线 K/X0/U0，并执行文档 §4.4 第 8 条的四项强校验 */
    void load_gain(const std::string& path, const std::string& robot_description)
    {
        if (control_.dt <= 0.0 || model_.wheel_radius <= 0.0)
        {
            throw std::invalid_argument("wheel_radius 和 dt 必须大于 0");
        }
        if (robot_description.empty())
        {
            throw std::runtime_error("robot_description 为空，无法校验增益对应的模型版本");
        }

        const YAML::Node root = YAML::LoadFile(path);
        const std::array<const char*, kStateDim> states{
            "s", "s_dot", "pitch", "pitch_rate", "q_h_common", "q_h_common_rate", "q_k_common", "q_k_common_rate"};
        const std::array<const char*, kInputDim> inputs{"tau_w_common", "tau_h_common", "tau_k_common"};
        require_layout(root["layout"]["states"], states, "layout.states");
        require_layout(root["layout"]["inputs"], inputs, "layout.inputs");

        const std::array<const char*, kRollStateDim> roll_states{
            "q_h_diff", "q_h_diff_rate", "q_k_diff", "q_k_diff_rate"};
        const std::array<const char*, kRollInputDim> roll_inputs{"tau_h_diff", "tau_k_diff"};
        require_layout(root["roll"]["layout"]["states"], roll_states, "roll.layout.states");
        require_layout(root["roll"]["layout"]["inputs"], roll_inputs, "roll.layout.inputs");

        const double gain_dt = root["model"]["dt"].as<double>();
        if (std::abs(gain_dt - control_.dt) > 1e-12)
        {
            throw std::runtime_error("增益 dt=" + std::to_string(gain_dt) + " 与 chassis dt=" + std::to_string(control_.dt) + " 不一致");
        }
        const double gain_radius = root["model"]["wheel_radius"].as<double>();
        if (std::abs(gain_radius - model_.wheel_radius) > 1e-12)
        {
            throw std::runtime_error("增益轮半径与 chassis 参数不一致");
        }

        const std::string expected_hash = root["model"]["hash"].as<std::string>();
        const std::string actual_hash = hash_string(robot_description);
        if (actual_hash != expected_hash)
        {
            throw std::runtime_error("增益模型哈希 " + expected_hash + " 与当前 URDF " + actual_hash + " 不一致，请重新生成增益");
        }

        const YAML::Node x0 = root["operating_point"]["x0"];
        const YAML::Node u0 = root["operating_point"]["u0"];
        const YAML::Node k = root["gain"]["k"];
        if (!root["operating_point"]["height"])
        {
            throw std::runtime_error("增益文件缺少 operating_point.height，不能用于高度调度");
        }
        loaded_gain_height_ = root["operating_point"]["height"].as<double>();
        if (!x0.IsSequence() || x0.size() != kStateDim || !u0.IsSequence() || u0.size() != kInputDim ||
            !k.IsSequence() || k.size() != kInputDim)
        {
            throw std::runtime_error("增益文件的 X0/U0/K 维度不对");
        }
        for (std::size_t i = 0; i < kStateDim; ++i)
        {
            x0_(static_cast<Eigen::Index>(i)) = x0[i].as<double>();
        }
        for (std::size_t row = 0; row < kInputDim; ++row)
        {
            u0_(static_cast<Eigen::Index>(row)) = u0[row].as<double>();
            if (!k[row].IsSequence() || k[row].size() != kStateDim)
            {
                throw std::runtime_error("增益文件 K 的第 " + std::to_string(row) + " 行维度不对");
            }
            for (std::size_t column = 0; column < kStateDim; ++column)
            {
                gain_(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) = k[row][column].as<double>();
            }
        }
        if (!std::isfinite(loaded_gain_height_) || !gain_.allFinite() || !x0_.allFinite() || !u0_.allFinite())
        {
            throw std::runtime_error("增益文件的 X0/U0/K 含 NaN 或 Inf");
        }

        const YAML::Node roll_x0 = root["roll"]["operating_point"]["x0"];
        const YAML::Node roll_u0 = root["roll"]["operating_point"]["u0"];
        const YAML::Node roll_per_q = root["roll"]["operating_point"]["roll_per_q"];
        if (!roll_x0.IsSequence() || roll_x0.size() != kRollStateDim ||
            !roll_u0.IsSequence() || roll_u0.size() != kRollInputDim ||
            !roll_per_q.IsSequence() || roll_per_q.size() != 2)
        {
            throw std::runtime_error("Roll 增益文件的 X0/U0/roll_per_q 维度不对");
        }
        for (std::size_t i = 0; i < kRollStateDim; ++i)
        {
            roll_x0_(static_cast<Eigen::Index>(i)) = roll_x0[i].as<double>();
        }
        for (std::size_t i = 0; i < kRollInputDim; ++i)
        {
            roll_u0_(static_cast<Eigen::Index>(i)) = roll_u0[i].as<double>();
            roll_per_q_(static_cast<Eigen::Index>(i)) = roll_per_q[i].as<double>();
        }

        roll_a_ = load_matrix<kRollStateDim, kRollStateDim>(root["roll"]["continuous"]["a"], "roll.continuous.a");
        roll_b_ = load_matrix<kRollStateDim, kRollInputDim>(root["roll"]["continuous"]["b"], "roll.continuous.b");
        roll_gain_ = load_matrix<kRollInputDim, kRollStateDim>(root["roll"]["gain"]["k"], "roll.gain.k");
        if (!roll_a_.allFinite() || !roll_b_.allFinite() || !roll_gain_.allFinite() ||
            !roll_x0_.allFinite() || !roll_u0_.allFinite() || !roll_per_q_.allFinite())
        {
            throw std::runtime_error("Roll 增益文件的 A/B/K/X0/U0/roll_per_q 含 NaN 或 Inf");
        }

        RCLCPP_INFO(get_logger(),
                    "已加载全身 LQR：%s，高度 %.4f m，模型 %s，Pitch U0=[%.6f, %.6f, %.6f]，Roll U0=[%.6f, %.6f]",
                    path.c_str(), loaded_gain_height_, actual_hash.c_str(),
                    u0_(0), u0_(1), u0_(2), roll_u0_(0), roll_u0_(1));
    }

    void apply_gain_schedule(double height)
    {
        if (gain_schedule_.empty())
        {
            throw std::runtime_error("增益调度表为空");
        }

        const GainPoint* lower = &gain_schedule_.front();
        const GainPoint* upper = lower;
        if (height >= gain_schedule_.back().height)
        {
            lower = &gain_schedule_.back();
            upper = lower;
        }
        else if (height > gain_schedule_.front().height)
        {
            for (std::size_t i = 1; i < gain_schedule_.size(); ++i)
            {
                if (height <= gain_schedule_[i].height)
                {
                    lower = &gain_schedule_[i - 1];
                    upper = &gain_schedule_[i];
                    break;
                }
            }
        }

        const double span = upper->height - lower->height;
        const double ratio = span > 0.0 ? std::clamp((height - lower->height) / span, 0.0, 1.0) : 0.0;
        const double inverse = 1.0 - ratio;
        gain_ = inverse * lower->gain + ratio * upper->gain;
        x0_ = inverse * lower->x0 + ratio * upper->x0;
        u0_ = inverse * lower->u0 + ratio * upper->u0;
        roll_gain_ = inverse * lower->roll_gain + ratio * upper->roll_gain;
        roll_x0_ = inverse * lower->roll_x0 + ratio * upper->roll_x0;
        roll_u0_ = inverse * lower->roll_u0 + ratio * upper->roll_u0;
        roll_per_q_ = inverse * lower->roll_per_q + ratio * upper->roll_per_q;
    }

    void validate_leg_params() const
    {
        constexpr std::size_t count = 4;
        if (leg_.joint_names.size() != count || leg_.command_topics.size() != count || leg_.max_effort.size() != count ||
            leg_.max_speed_radps.size() != count || leg_.limit_lower.size() != count || leg_.limit_upper.size() != count ||
            leg_.joint_sign.size() != count || leg_.effort_rate_limit.size() != count ||
            leg_.soft_limit_margin.size() != count || leg_.soft_limit_kp.size() != count || leg_.soft_limit_kd.size() != count)
        {
            throw std::invalid_argument("腿参数数组必须都是 4 项，顺序 [左髋,右髋,左膝,右膝]");
        }
        for (std::size_t i = 0; i < count; ++i)
        {
            if (leg_.limit_lower[i] >= leg_.limit_upper[i] || leg_.max_effort[i] <= 0.0 ||
                leg_.effort_rate_limit[i] <= 0.0 || leg_.soft_limit_margin[i] <= 0.0 ||
                2.0 * leg_.soft_limit_margin[i] >= leg_.limit_upper[i] - leg_.limit_lower[i] ||
                leg_.soft_limit_kp[i] < 0.0 || leg_.soft_limit_kd[i] < 0.0)
            {
                throw std::invalid_argument("腿关节力矩、变化率或软硬限位参数无效：" + leg_.joint_names[i]);
            }
        }
        if (control_.wheel_effort_rate_limit <= 0.0 || safety_.feedback_timeout_s <= 0.0 ||
            safety_.max_control_period_s <= control_.dt || safety_.recovery_dwell_s < 0.0)
        {
            throw std::invalid_argument("力矩变化率或安全监督时间参数无效");
        }
    }

    void configure_allocator()
    {
        TorqueAllocator::Config config;
        config.dt = control_.dt;
        const double wheel_limit = control_.max_effort + remote_.max_differential;
        config.effort_limit = {wheel_limit, wheel_limit,
                               leg_.max_effort[0], leg_.max_effort[1], leg_.max_effort[2], leg_.max_effort[3]};
        config.effort_rate_limit = {control_.wheel_effort_rate_limit, control_.wheel_effort_rate_limit, leg_.effort_rate_limit[0],
                                    leg_.effort_rate_limit[1],        leg_.effort_rate_limit[2],        leg_.effort_rate_limit[3]};
        for (std::size_t i = 0; i < TorqueAllocator::kLegCount; ++i)
        {
            config.hard_lower[i]    = leg_.limit_lower[i];
            config.hard_upper[i]    = leg_.limit_upper[i];
            config.soft_lower[i]    = leg_.limit_lower[i] + leg_.soft_limit_margin[i];
            config.soft_upper[i]    = leg_.limit_upper[i] - leg_.soft_limit_margin[i];
            config.soft_limit_kp[i] = leg_.soft_limit_kp[i];
            config.soft_limit_kd[i] = leg_.soft_limit_kd[i];
        }
        allocator_ = std::make_unique<TorqueAllocator>(config);
    }

    void configure_height_kinematics()
    {
        if (!std::isfinite(height_.link_length) || height_.link_length <= 0.0 ||
            !std::isfinite(height_.link_angle) ||
            !std::isfinite(height_.maximum_rate_mps) || height_.maximum_rate_mps <= 0.0 ||
            !std::isfinite(height_.rod_mass) || height_.rod_mass <= 0.0 ||
            !std::isfinite(height_.joint_mass) || height_.joint_mass <= 0.0 ||
            !std::isfinite(height_.wheel_mass) || height_.wheel_mass <= 0.0)
        {
            throw std::invalid_argument("腿高几何、质量和变化率必须为有限正数");
        }
        if (!std::isfinite(remote_.height_step_m) || remote_.height_step_m <= 0.0)
        {
            throw std::invalid_argument("height_step_m 必须是有限正数");
        }

        LegHeightKinematics::Config config;
        config.link_length = height_.link_length;
        config.nominal_link_angle = height_.link_angle;
        config.hip_lower   = std::max(leg_.limit_lower[0] + leg_.soft_limit_margin[0],
                                      leg_.limit_lower[1] + leg_.soft_limit_margin[1]);
        config.hip_upper   = std::min(leg_.limit_upper[0] - leg_.soft_limit_margin[0],
                                      leg_.limit_upper[1] - leg_.soft_limit_margin[1]);
        config.knee_lower  = std::max(leg_.limit_lower[2] + leg_.soft_limit_margin[2],
                                      leg_.limit_lower[3] + leg_.soft_limit_margin[3]);
        config.knee_upper  = std::min(leg_.limit_upper[2] - leg_.soft_limit_margin[2],
                                      leg_.limit_upper[3] - leg_.soft_limit_margin[3]);
        height_kinematics_ = std::make_unique<LegHeightKinematics>(config);

        if (!std::isfinite(height_.target_height_m) ||
            height_.target_height_m < height_kinematics_->minimum_height() ||
            height_.target_height_m > height_kinematics_->maximum_height())
        {
            throw std::invalid_argument("target_height_m 超出软限位可达范围");
        }
        height_reference_ = height_kinematics_->from_height(height_kinematics_->nominal_height());
    }

    static double rotated_x(double angle, double x, double z)
    {
        return x * std::cos(angle) + z * std::sin(angle);
    }

    std::pair<double, double> height_gravity_effort(double hip, double knee) const
    {
        const double span = height_.link_length * std::cos(height_.link_angle);
        const double drop = height_.link_length * std::sin(height_.link_angle);
        const double knee_x = rotated_x(hip, span, -drop);
        const double wheel_x = knee_x + rotated_x(hip + knee, -span, -drop);
        const double thigh_com_x = rotated_x(hip, 0.5 * span, -0.5 * drop);
        const double calf_com_x = knee_x + rotated_x(hip + knee, -0.5 * span, -0.5 * drop);

        const double normal_force = 0.5 * (model_.body_mass + 2.0 * height_.wheel_mass) * model_.gravity;
        const double rod_weight = -height_.rod_mass * model_.gravity;
        const double wheel_weight = -height_.wheel_mass * model_.gravity;
        const double joint_weight = -height_.joint_mass * model_.gravity;
        const auto moment = [](double offset_x, double force_z) { return -offset_x * force_z; };

        const double knee_external = moment(wheel_x - knee_x,       normal_force)
                                   + moment(calf_com_x - knee_x,    rod_weight)
                                   + moment(wheel_x - knee_x,       wheel_weight);
        const double hip_external  = moment(wheel_x,                normal_force)
                                   + moment(thigh_com_x,            rod_weight)
                                   + moment(knee_x,                 joint_weight)
                                   + moment(calf_com_x,             rod_weight)
                                   + moment(wheel_x,                wheel_weight);
        return {-hip_external, -knee_external};
    }

    /**
     * @brief 收到一帧 IMU：取出横滚/俯仰角及角速度（只用 orientation 和 angular_velocity）。
     *        仿真里 orientation 是位姿真值、不漂，实车要换成 imu 节点的融合输出（教程 §3.3）
     */
    void on_imu(const sensor_msgs::msg::Imu& msg)
    {
        const double w = msg.orientation.w;
        const double x = msg.orientation.x;
        const double y = msg.orientation.y;
        const double z = msg.orientation.z;

        // ZYX 欧拉角里的 roll 分量。Roll 四状态模型已消去这个约束坐标，这里用于安全和闭链一致性诊断。
        state_.roll       = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        state_.roll_rate  = msg.angular_velocity.x;

        // 绕 +y 的俯仰角（ZYX 欧拉角里的 pitch 分量）。clamp 是防浮点越界让 asin 出 NaN
        const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
        state_.pitch      = control_.pitch_sign * std::asin(sin_pitch);
        state_.pitch_rate = control_.pitch_sign * msg.angular_velocity.y;
        state_.yaw_rate   = msg.angular_velocity.z;
        flags_.has_imu    = true;
        last_imu_time_    = std::chrono::steady_clock::now();
    }

    /**
     * @brief 收到一帧关节状态：挑出两个轮关节，折成整车的位移和速度（两个都在这一帧里才认）
     *
     * @note position 是**累积角**（continuous 关节一直涨），只是"相对原点的里程"，
     *       原点在 update() 里标定。忘了减原点，车一起步就以为自己在很远的地方
     */
    void on_joint_state(const sensor_msgs::msg::JointState& msg)
    {
        // name / position / velocity 三个数组不保证一样长，一起截短，别各自判越界
        const std::size_t count = std::min({msg.name.size(), msg.position.size(), msg.velocity.size()});

        double      position_sum = 0.0;
        double      velocity_sum = 0.0;
        std::size_t wheel_count  = 0;

        // 腿关节先全置无效，找到哪个再置回来：宁可这拍当没有，也别拿上一帧的旧角度当现值
        state_.leg_valid.fill(false);
        flags_.has_joint_state = false;

        for (std::size_t i = 0; i < count; ++i)
        {
            if (msg.name[i] == joint_names_[kLeft] || msg.name[i] == joint_names_[kRight])
            {
                position_sum += msg.position[i];
                velocity_sum += msg.velocity[i];
                ++wheel_count;
                continue;
            }

            for (std::size_t leg = 0; leg < leg_.joint_names.size(); ++leg)
            {
                if (msg.name[i] == leg_.joint_names[leg])
                {
                    state_.leg_position_rad[leg]  = msg.position[i];
                    state_.leg_velocity_radps[leg] = msg.velocity[i];
                    state_.leg_valid[leg]         = true;
                    break;
                }
            }
        }

        const bool legs_complete = std::all_of(state_.leg_valid.begin(), state_.leg_valid.end(), [](bool valid) { return valid; });
        if (wheel_count != kWheelCount || !legs_complete)
        {
            return;
        }

        state_.wheel_position  = model_.wheel_radius * position_sum * 0.5;
        state_.wheel_velocity  = model_.wheel_radius * velocity_sum * 0.5;
        flags_.has_joint_state = true;
        last_joint_state_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 收到一条遥控指令：限幅之后当成目标，别的一概不管
     *
     * @note 先用 isfinite 挡一遍再 clamp：std::clamp(NaN, ...) 不保证挡得住，
     *       一个 NaN 进来整个状态就是 NaN，表现是车突然不动了或者抽一下
     */
    void on_cmd_vel(const geometry_msgs::msg::Twist& msg)
    {
        if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.linear.z) ||
            !std::isfinite(msg.angular.z))
        {
            return;   // 丢掉这一条，留着上一条；之后超时了照样会停车
        }

        // 进来的是归一化方向（±1 = 满速），实际多快由这边定：乘上限，顺手限幅
        teleop_.velocity = remote_.max_velocity * std::clamp(msg.linear.x,  -1.0, 1.0);
        teleop_.yaw_rate = remote_.max_yaw_rate * std::clamp(msg.angular.z, -1.0, 1.0);

        // linear.z 只是一拍的离散事件，不是持续速度：键盘每识别到一次 '=' / '-'
        // 分别发 +1 / -1。这里修改目标高度，真正的关节目标仍由 height_rate_mps 平滑推进。
        const double height_direction = std::clamp(msg.linear.z, -1.0, 1.0);
        if (height_direction != 0.0)
        {
            const double requested_height = std::clamp(
                height_.target_height_m + height_direction * remote_.height_step_m,
                height_kinematics_->minimum_height(), height_kinematics_->maximum_height());
            if (requested_height != height_.target_height_m)
            {
                const auto result = set_parameter(rclcpp::Parameter("target_height_m", requested_height));
                if (!result.successful)
                {
                    RCLCPP_WARN(get_logger(), "键盘身高目标更新失败：%s", result.reason.c_str());
                }
            }
        }

        flags_.has_command = true;
        last_command_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 最新一条指令到现在过了多久（没收到过时返回无穷大）
     *
     * @note 用 steady_clock 而不是仿真时间：仿真会被暂停/复位，拿它算会判错
     */
    double command_age_s() const
    {
        if (!flags_.has_command)
        {
            return std::numeric_limits<double>::infinity();
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_time_).count();
    }

    static const char* safety_state_name(SafetyState state)
    {
        switch (state)
        {
            case SafetyState::kWaitingFeedback: return "等待反馈";
            case SafetyState::kRecovering:      return "恢复确认";
            case SafetyState::kActive:          return "正常控制";
            case SafetyState::kFeedbackTimeout: return "反馈超时";
            case SafetyState::kInvalidState:    return "状态非有限";
            case SafetyState::kFallen:          return "姿态倒地";
            case SafetyState::kJointLimit:      return "关节越过硬限位";
            case SafetyState::kTimingFault:     return "控制周期异常";
        }
        return "未知";
    }

    bool state_is_finite() const
    {
        if (!std::isfinite(state_.roll)     || !std::isfinite(state_.roll_rate)      ||
            !std::isfinite(state_.pitch)    || !std::isfinite(state_.pitch_rate)     ||
            !std::isfinite(state_.yaw_rate) || !std::isfinite(state_.wheel_position) ||
            !std::isfinite(state_.wheel_velocity))
        {
            return false;
        }
        for (std::size_t i = 0; i < state_.leg_position_rad.size(); ++i)
        {
            if (!std::isfinite(state_.leg_position_rad[i]) || !std::isfinite(state_.leg_velocity_radps[i]))
            {
                return false;
            }
        }
        return true;
    }

    SafetyState evaluate_safety(const std::chrono::steady_clock::time_point& now, bool timing_fault) const
    {
        if (!flags_.has_imu || !flags_.has_joint_state)
        {
            return SafetyState::kWaitingFeedback;
        }
        if (std::chrono::duration<double>(now - last_imu_time_).count() > safety_.feedback_timeout_s ||
            std::chrono::duration<double>(now - last_joint_state_time_).count() > safety_.feedback_timeout_s)
        {
            return SafetyState::kFeedbackTimeout;
        }
        if (timing_fault)
        {
            return SafetyState::kTimingFault;
        }
        if (!state_is_finite())
        {
            return SafetyState::kInvalidState;
        }
        if (std::abs(state_.pitch) > control_.fall_angle_rad || std::abs(state_.roll) > control_.fall_angle_rad)
        {
            return SafetyState::kFallen;
        }
        for (std::size_t i = 0; i < state_.leg_position_rad.size(); ++i)
        {
            if (state_.leg_position_rad[i] < leg_.limit_lower[i] || state_.leg_position_rad[i] > leg_.limit_upper[i])
            {
                return SafetyState::kJointLimit;
            }
        }
        return SafetyState::kActive;
    }

    void reset_dynamic_state()
    {
        teleop_ = TeleopTarget{};
        flags_.origin_initialized = false;
        state_.wheel_velocity = 0.0;
        allocator_->reset();
        double reset_height = height_kinematics_->nominal_height();
        if (std::isfinite(state_.leg_position_rad[0]) && std::isfinite(state_.leg_position_rad[1]))
        {
            const double common_hip = 0.5 * (state_.leg_position_rad[0] + state_.leg_position_rad[1]);
            reset_height = height_kinematics_->height_from_hip(common_hip);
        }
        height_reference_ = height_kinematics_->from_height(reset_height);
    }

    void enter_safe_state(SafetyState reason)
    {
        if (safety_state_ != reason)
        {
            RCLCPP_WARN(get_logger(), "安全状态：%s，六路力矩立即清零", safety_state_name(reason));
        }
        safety_state_ = reason;
        has_healthy_since_ = false;
        reset_dynamic_state();
        publish_zero_effort();
    }

    /**
     * @brief 控制节拍：把最新一帧反馈喂给 LQR，把力矩发给两个轮子
     *        （回调只缓存，整形和求解都在这一拍里做，跟话题频率解耦）
     */
    void update()
    {
        const auto now = std::chrono::steady_clock::now();
        bool timing_fault = false;
        if (has_update_time_)
        {
            const double period = std::chrono::duration<double>(now - last_update_time_).count();
            timing_fault = period > safety_.max_control_period_s;
        }
        last_update_time_ = now;
        has_update_time_ = true;

        const SafetyState safety_result = evaluate_safety(now, timing_fault);
        if (safety_result != SafetyState::kActive)
        {
            enter_safe_state(safety_result);
            return;
        }

        // 故障刚消失时持续观察一段时间，再一次性重置并交还 LQR，避免边界附近每拍抢控制权。
        if (safety_state_ != SafetyState::kActive)
        {
            // 首次完整反馈必须立即接管；机器人不能在自重下空等恢复驻留时间。
            if (!ever_active_)
            {
                reset_dynamic_state();
                safety_state_ = SafetyState::kActive;
                ever_active_ = true;
                RCLCPP_INFO(get_logger(), "首次反馈完整：标定位移原点并立即启用 LQR");
            }
            else
            {
                if (!has_healthy_since_)
                {
                    healthy_since_ = now;
                    has_healthy_since_ = true;
                    safety_state_ = SafetyState::kRecovering;
                    RCLCPP_INFO(get_logger(), "反馈恢复，持续健康 %.3f s 后重新接管", safety_.recovery_dwell_s);
                }
                if (std::chrono::duration<double>(now - healthy_since_).count() < safety_.recovery_dwell_s)
                {
                    publish_zero_effort();
                    return;
                }
                reset_dynamic_state();
                safety_state_ = SafetyState::kActive;
                RCLCPP_INFO(get_logger(), "安全状态恢复：重新标定位移原点并启用 LQR");
            }
        }

        if (!flags_.origin_initialized)
        {
            state_.position_origin    = state_.wheel_position;
            flags_.origin_initialized = true;
        }

        // 指令超时（遥控没在发）就当停车：别保留上一次的速度指令，断了不会一直冲
        if (command_age_s() > remote_.command_timeout_s)
        {
            teleop_.velocity = 0.0;
            teleop_.yaw_rate = 0.0;
        }

        // 让 LQR 追速度，位置那一路只是给位移项一个参照：死盯一个位移点的话，
        // 车为了"停在原地"会顶着速度指令不动
        teleop_.position += teleop_.velocity * control_.dt;

        const LegHeightKinematics::Reference previous_height_reference = height_reference_;
        const double commanded_height = LegHeightKinematics::move_towards(
            previous_height_reference.height, height_.target_height_m, height_.maximum_rate_mps, control_.dt);
        height_reference_ = height_kinematics_->from_height(commanded_height);
        apply_gain_schedule(height_reference_.height);
        const double target_hip_rate = (height_reference_.hip - previous_height_reference.hip) / control_.dt;
        const double target_knee_rate = -2.0 * target_hip_rate;

        State target = x0_;
        target(0) += teleop_.position;
        target(1) += teleop_.velocity;
        target(4) = height_reference_.hip;
        target(5) = target_hip_rate;
        target(6) = height_reference_.knee;
        target(7) = target_knee_rate;

        const double hip_position = 0.5 * (state_.leg_position_rad[0] + state_.leg_position_rad[1]);
        const double hip_velocity = 0.5 * (state_.leg_velocity_radps[0] + state_.leg_velocity_radps[1]);
        const double knee_position = 0.5 * (state_.leg_position_rad[2] + state_.leg_position_rad[3]);
        const double knee_velocity = 0.5 * (state_.leg_velocity_radps[2] + state_.leg_velocity_radps[3]);

        const double hip_difference = 0.5 * (state_.leg_position_rad[0] - state_.leg_position_rad[1]);
        const double hip_difference_rate = 0.5 * (state_.leg_velocity_radps[0] - state_.leg_velocity_radps[1]);
        const double knee_difference = 0.5 * (state_.leg_position_rad[2] - state_.leg_position_rad[3]);
        const double knee_difference_rate = 0.5 * (state_.leg_velocity_radps[2] - state_.leg_velocity_radps[3]);

        State state;
        state << state_.wheel_position - state_.position_origin, state_.wheel_velocity,
                 state_.pitch, state_.pitch_rate,
                 hip_position, hip_velocity, knee_position, knee_velocity;

        // U = U0 - K(X-Xref)。U 里的共模量是"每侧执行器各出这么多"，拆回左右时不再除 2。
        Control feedforward = u0_;
        const auto [height_hip_u0, height_knee_u0] =
            height_gravity_effort(height_reference_.hip, height_reference_.knee);
        feedforward(1) = height_hip_u0;
        feedforward(2) = height_knee_u0;
        Control common = feedforward - control_.control_sign * gain_ * (state - target);
        if (!enabled_.wheel_common) common(0) = 0.0;
        if (!enabled_.hip_common) common(1) = 0.0;
        if (!enabled_.knee_common) common(2) = 0.0;

        RollState roll_state;
        roll_state << hip_difference, hip_difference_rate, knee_difference, knee_difference_rate;
        RollControl leg_difference = roll_u0_ - control_.control_sign * roll_gain_ * (roll_state - roll_x0_);
        if (!enabled_.hip_diff) leg_difference(0) = 0.0;
        if (!enabled_.knee_diff) leg_difference(1) = 0.0;
        const double predicted_roll = (roll_per_q_ * Eigen::Vector2d(hip_difference, knee_difference))(0);
        const double predicted_roll_rate =
            (roll_per_q_ * Eigen::Vector2d(hip_difference_rate, knee_difference_rate))(0);

        // 转向：左右轮反向偏置，力矩差产生偏航力矩（正 = 左转）。反馈只有角速度、没有偏航角，
        // 所以这条环管的是"转多快"，不是"转到哪"
        const double differential = std::clamp(remote_.yaw_kp * (teleop_.yaw_rate - state_.yaw_rate), -remote_.max_differential, remote_.max_differential);

        TorqueAllocator::Input allocation_input;
        allocation_input.wheel_common = common(0);
        allocation_input.wheel_diff = -differential;  // 保持原偏航接线：左=c-d、右=c+d
        allocation_input.hip_common = common(1);
        allocation_input.hip_diff = leg_difference(0);
        allocation_input.knee_common = common(2);
        allocation_input.knee_diff = leg_difference(1);

        TorqueAllocator::Feedback allocation_feedback;
        allocation_feedback.position = state_.leg_position_rad;
        allocation_feedback.velocity = state_.leg_velocity_radps;
        const TorqueAllocator::Result allocation = allocator_->allocate(allocation_input, allocation_feedback);
        if (allocation.diagnostics.invalid_input)
        {
            enter_safe_state(SafetyState::kInvalidState);
            return;
        }
        publish_allocated_effort(allocation.effort);

        if (allocation.diagnostics.priority_limited || allocation.diagnostics.effort_limited ||
            allocation.diagnostics.rate_limited ||
            allocation.diagnostics.soft_limit_active)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                                 "力矩分配：共模优先裁剪=%d，成组限幅=%d(scale %.3f)，变化率=%d(scale %.3f)，软限位=%d",
                                 allocation.diagnostics.priority_limited,
                                 allocation.diagnostics.effort_limited, allocation.diagnostics.effort_scale,
                                 allocation.diagnostics.rate_limited, allocation.diagnostics.rate_scale,
                                 allocation.diagnostics.soft_limit_active);
        }

        // 六个执行器的诊断：轮子那两路上面已经在发，这里把四个腿关节的反馈打出来 ——
        // 阶段 1 验收看的就是"关节名对上了、位置速度在更新"。节流到 1 s 一条
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "姿态 pitch/roll %.4f/%.4f，腿高 当前/指令/目标 %.4f/%.4f/%.4f m，腿差模 h/k %.4f/%.4f，Roll角预测/残差 %.4f/%.4f，角速预测/残差 %.3f/%.3f，力矩共模 w/h/k %.4f/%.4f/%.4f，差模 h/k %.4f/%.4f",
                             state_.pitch, state_.roll,
                             height_kinematics_->height_from_hip(hip_position), height_reference_.height,
                             height_.target_height_m, hip_difference, knee_difference,
                             predicted_roll, state_.roll - predicted_roll,
                             predicted_roll_rate, state_.roll_rate - predicted_roll_rate,
                             common(0), common(1), common(2), leg_difference(0), leg_difference(1));
    }

    void publish_zero_effort()
    {
        std_msgs::msg::Float64 zero;
        for (auto& publisher : command_pub_)
        {
            publisher->publish(zero);
        }
        for (auto& publisher : leg_command_pub_)
        {
            publisher->publish(zero);
        }
    }

    void publish_allocated_effort(const std::array<double, TorqueAllocator::kActuatorCount>& effort)
    {
        for (std::size_t i = 0; i < kWheelCount; ++i)
        {
            std_msgs::msg::Float64 command;
            command.data = effort[i];
            command_pub_[i]->publish(command);
        }
        for (std::size_t i = 0; i < leg_command_pub_.size(); ++i)
        {
            std_msgs::msg::Float64 command;
            command.data = leg_.joint_sign[i] * effort[i + kWheelCount];
            leg_command_pub_[i]->publish(command);
        }
    }
};
