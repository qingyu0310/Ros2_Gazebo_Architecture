/**
 * @file chassis.hpp
 * @author qingyu
 * @brief 底盘节点：轮腿车的平衡控制器 —— 轮子状态 + 车身姿态 -> 两个轮子的力矩
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 * @note 分工：求解器在 framework（algorithm::controller::Lqr），单轮的限幅/速度环在
 *       ros2_layer 的 MotorNode 里，再往下是 framework 的 modules::motor::Motor。
 *       本节点只干"底盘"这一层的事：把状态喂给 LQR、把力矩按偏航差分分给两个轮子。
 *
 * @note 现在这个文件里有：参数 + 求解（A/B -> K）+ 三个订阅的回调（/imu、/joint_states、
 *       /chassis/cmd_vel）+ 控制节拍 + 两个电机子节点（外加把子节点交给 executor 的 nodes()）。
 *       力矩分两路：公共的一份管"站住 + 前后走"，左右轮的差速那一份管"转"。
 *
 * @note 参数按用途分了四个结构体（ModelParams / ControlParams / RemoteParams /
 *       TopicParams），那是**代码里**的分组；参数文件里还是平铺的一层，
 *       下面 declare_parameter 的名字就是 yaml 里的键名，没有嵌套。
 * @note 运行时的量也分了三组：ChassisState（反馈）、TeleopTarget（目标）、StatusFlags
 *       （标志位）。它们是"每拍在变的"，跟上面那组"启动时读一次的参数"不是一回事
 *
 * @note 模型参数是量出来的，不是拍的：见 docs/轮腿机器人LQR平衡教程.md §1.3 那段脚本，
 *       从 robot/models/bodys/wheel_leg_robot.xacro 展开出的 URDF 里算。改模型就要重算
 *       —— 这些数对不上不报错，只是站不住。
 *       腿一伸缩（modules/leg.xacro 里 fixed 换 revolute）这组就失效：l 和 I 跟着质心走，
 *       A/B 全变，得按高度重算一组 K。
 *
 * @note 话题（默认值，都是参数）：
 *         订阅 /chassis/cmd_vel       geometry_msgs/Twist，只读 linear.x 和 angular.z
 *         订阅 /imu                   sensor_msgs/Imu，要 orientation 和 angular_velocity
 *         订阅 /joint_states          sensor_msgs/JointState，要两个轮关节的 position/velocity
 *         发布 /motor/<轮子>/command   std_msgs/Float64，effort 模式，单位 N·m
 *
 * @note 关节名 / 指令话题 / 力矩话题**只写在构造函数里当默认值**，不进 params 文件：
 *       那是"这台车怎么接线"，由底盘经 parameter_overrides 覆盖给两个电机子节点
 * @note 两个电机在同一个进程里：同一台车的两个轮子必须由同一条指令同时算出来，
 *       拆成两个进程还得再搞一套同步。子节点由同一个 executor 一起 spin，见 nodes()
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "framework/algorithm/controller/lqr.hpp"
#include "ros2_layer/node/motor/motor.hpp"

/**
 * @brief 模型参数：这台车的物理量，由 URDF 量出来（见本文件头的说明）
 *
 * @note 前五个是 LQR 的输入（r/M/m/l/I），b 是阻尼，gravity 是常数。
 *       字段名后面没有下划线：下划线在本项目里是"节点类的成员"的标记，它们不是
 */
struct ModelParams
{
    double wheel_radius       {0.040};                                                  // r，跟 modules/wheel.xacro 的 radius 对齐
    double cart_mass          {0.6375};                                                 // M，两轮质量 + 各自绕轴惯量/r² 的折合
    double body_mass          {1.25};                                                   // m，除轮子外的车身质量
    double body_com_height    {0.06825};                                                // l，车身质心到轮轴的距离
    double body_pitch_inertia {0.002591};                                               // I，车身绕质心、绕 y 轴的俯仰惯量
    double cart_damping       {0.02};                                                   // b，等效粘性阻尼，估的量
    double gravity            {9.81};
};

/**
 * @brief 控制参数：离散化步长、LQR 权重、限幅与符号
 *
 * @note mode 也归这里：它决定电机吃的是力矩还是速度，是控制决策而不是接线
 */
struct ControlParams
{
    double              dt             {0.001};                                         // 控制周期，要跟物理步长和定时器实际周期一致
    std::vector<double> q_weights      {1.0, 1.0, 1000.0, 550.0};       // [位移, 速度, 俯仰, 俯仰角速度]，长度必须是 4
    double              r_weight       {1.0};                                           // 单轮力矩的惩罚权重，越大越柔和
    double              max_effort     {0.25};                                          // 公共力矩上限 N·m（附着极限 0.30）
    double              fall_angle_rad {0.70};                                          // |俯仰| 超过它就认定倒地，力矩归零
    double              pitch_sign     {1.0};                                           // 俯仰取反用 -1.0
    double              control_sign   {1.0};                                           // 力矩取反用 -1.0
    std::string         mode           {"effort"};                                   // 只能是 effort，速度环会和平衡环抢
};

/**
 * @brief 遥控指令接口：linear.x 前后速度（正=前进），angular.z 偏航角速度（正=左转）
 *
 * @note yaw_kp / max_differential / yaw_sign 三个是转向环的：偏航角速度误差 -> 左右轮
 *       力矩差 -> 偏航力矩。方向反了翻 yaw_sign，别去翻 kp 的符号
 * @note linear_sign 只管"遥控按前进、车往哪边跑"：方向反了翻成 -1.0。它乘的是目标值，
 *       状态和力矩那两路的符号不动 —— 平衡环是自洽的，翻不动它，也别去翻
 * @note command_timeout_s 是安全项：这么久没收到指令就当停车，遥控断了不会保持上一次的速度
 */
struct RemoteParams
{
    std::string cmd_vel_topic     {"/chassis/cmd_vel"};
    double      max_velocity      {0.20};
    double      linear_sign       {1.0};                                                // 前进方向反了翻成 -1.0
    double      max_yaw_rate      {1.0};
    double      yaw_kp            {0.01};                                               // 偏航误差 -> 力矩差 N·m·s/rad
    double      max_differential  {0.02};                                               // 力矩差上限，单轮上限 = 它 + max_effort
    double      yaw_sign          {1.0};                                                // 转向方向反了翻成 -1.0
    double      command_timeout_s {0.30};                                               // 这么久没收到 cmd_vel 就当停车
};

/**
 * @brief 反馈话题：两个订阅从哪一路来
 *
 * @note 名字要跟模型里对得上：imu_topic 对应 gazebo/plugins/sensor/imu.xacro 的 topic，
 *       joint_states_topic 对应 joint_encoder 宏的 topic，两边默认值都是现在这两个
 */
struct TopicParams
{
    std::string imu_topic          {"/imu"};
    std::string joint_states_topic {"/joint_states"};
};

/**
 * @brief 底盘状态：最近一帧反馈里挑出来的量，前四个就是 LQR 的 x
 *
 * @note 取的是两个轮子的**平均**：一起往前走时各贡献一半，差的那一半是偏航，不算进里程
 * @note position_origin 是标定用的：continuous 关节报的是累积角，减掉原点才是里程。
 *       倒地那段时间轮子在空转，扶正之后原点要重标（见 update()）
 * @note 字段名后面没有下划线：下划线在本项目里是"节点类的成员"的标记，它们不是
 */
struct ChassisState
{
    double pitch           {0.0};   // θ，车身俯仰角，前倾为正
    double pitch_rate      {0.0};   // θ̇
    double yaw_rate        {0.0};   // 偏航角速度，只给转向环用
    double wheel_position  {0.0};   // r·(pos_L+pos_R)/2，轮子转过的角折成"走了多远"
    double wheel_velocity  {0.0};   // ṗ
    double position_origin {0.0};   // 位移原点：减掉它才是里程
};

/**
 * @brief 遥控目标：cmd_vel 解出来的两个目标值，外加一个由速度积分出来的位置目标
 *
 * @note position 是"速度目标的积分"而不是遥控直接给的：LQR 追的是速度，位置那一路只是
 *       给位移项一个参照。死盯一个位移点的话，车为了"停在原地"会顶着速度指令不动
 * @note 超时判断用的那个时刻戳不在这里：它是时间不是目标值，单独放下面
 */
struct TeleopTarget
{
    double velocity {0.0};   // 目标前进速度 m/s，来自 cmd_vel 的 linear.x
    double yaw_rate {0.0};   // 目标偏航角速度 rad/s，来自 cmd_vel 的 angular.z
    double position {0.0};   // 位置目标，由 velocity 积分而来
};

/**
 * @brief 标志位：几路反馈来没来、位移原点标定没标定、上一拍是不是判了倒地
 *
 * @note 反馈标志必须先置上才发力：拿 0 当状态等于盲发力（见 update()）
 * @note fallen 记的是**上一拍**的判断，用来抓"刚被扶正"那一刻 —— 那一刻要重标位移原点
 */
struct StatusFlags
{
    bool has_imu            {false};   // 收到过 IMU 没有
    bool has_joint_state    {false};   // 收到过"两个轮子都在"的一帧没有
    bool origin_initialized {false};   // 位移原点标定过没有
    bool fallen             {false};   // 上一拍是不是已经判了倒地（用来抓"刚被扶正"那一刻）
    bool has_command        {false};   // 收到过 cmd_vel 没有
};

/**
 * @brief 轮腿车底盘：两轮倒立摆的平衡控制器
 *
 * @note 状态取四个 [轮子位移, 轮子速度, 车身俯仰, 俯仰角速度]，控制只有一个 ——
 *       两个轮子的**公共**力矩（N·m）；左右轮方向差的那一份留给偏航
 * @note 力矩模式（control_.mode = effort）是这套控制的前提：平衡环要的是"给我这个力矩"，
 *       中间插一层速度环等于多一个积分器 + 一层延迟，两个环会互相顶
 */
class ChassisNode : public rclcpp::Node
{
public:
    // 4 状态 1 输入：x = [轮子位移, 轮子速度, 俯仰, 俯仰角速度]，u = 单轮力矩（两轮共用一份）
    using Controller    = algorithm::controller::Lqr<4, 1, double>;
    using State         = Controller::State;
    using StateMatrix   = Controller::StateMatrix;
    using InputMatrix   = Controller::InputMatrix;
    using ControlMatrix = Controller::ControlMatrix;

    // 轮子下标。数组全按这个顺序对齐，别再散着写
    static constexpr std::size_t kLeft       = 0;
    static constexpr std::size_t kRight      = 1;
    static constexpr std::size_t kWheelCount = 2;

    ChassisNode() : rclcpp::Node("chassis")
    {
        // ---- 模型参数：由 URDF 量出来，见 docs/轮腿机器人LQR平衡教程.md §1.3 ----
        model_.wheel_radius       = declare_parameter("wheel_radius",       0.040);
        model_.cart_mass          = declare_parameter("cart_mass",          0.6375);
        model_.body_mass          = declare_parameter("body_mass",          1.25);
        model_.body_com_height    = declare_parameter("body_com_height",    0.06825);
        model_.body_pitch_inertia = declare_parameter("body_pitch_inertia", 0.002591);
        model_.cart_damping       = declare_parameter("cart_damping",       0.02);
        model_.gravity            = declare_parameter("gravity",            9.81);

        // ---- 控制 ----
        control_.dt             = declare_parameter("dt",                         0.001);
        control_.q_weights      = declare_parameter<std::vector<double>>("q",     {1.0, 1.0, 1000.0, 550.0});
        control_.r_weight       = declare_parameter("r",                          1.0);
        control_.max_effort     = std::abs(declare_parameter("max_effort",     0.25));
        control_.fall_angle_rad = std::abs(declare_parameter("fall_angle_rad", 0.70));
        control_.pitch_sign     = declare_parameter("pitch_sign",                 1.0);
        control_.control_sign   = declare_parameter("control_sign",               1.0);
        control_.mode           = declare_parameter("control_mode",               "effort");

        // 力矩模式是这套控制的前提：平衡环要的是"给我这个力矩"，中间插一层速度环
        // 等于多一个积分器 + 一层延迟，两个环会互相顶。这个值还会覆盖给两个电机，
        // 所以在这里就拦下来，别等电机把力矩当转速读
        if (control_.mode != "effort")
        {
            throw std::invalid_argument("平衡环必须用 effort 模式：control_mode 只能是 'effort'");
        }

        // ---- 遥控接口 ----
        remote_.cmd_vel_topic     = declare_parameter("cmd_vel_topic",                 "/chassis/cmd_vel");
        remote_.max_velocity      = std::abs(declare_parameter("max_velocity",      0.20));
        remote_.linear_sign       = declare_parameter("linear_sign",                   1.0);
        remote_.max_yaw_rate      = std::abs(declare_parameter("max_yaw_rate",      1.0));
        remote_.yaw_kp            = std::abs(declare_parameter("yaw_kp",            0.01));
        remote_.max_differential  = std::abs(declare_parameter("max_differential",  0.02));
        remote_.yaw_sign          = declare_parameter("yaw_sign",                      1.0);
        remote_.command_timeout_s = std::abs(declare_parameter("command_timeout_s", 0.30));

        // ---- 反馈话题 ----
        topics_.imu_topic          = declare_parameter("imu_topic",          "/imu");
        topics_.joint_states_topic = declare_parameter("joint_states_topic", "/joint_states");

        // ---- 求解 ----
        configure_lqr();   // 参数 -> A/B -> K，一次性；之后每拍只调 controller_.update()

        // ---- 两个轮子的接线：哪个关节、指令发到哪个话题、力矩从哪个话题出去 ----
        // 三组都按下标跟 kLeft / kRight 对齐
        joint_names_    = {declare_parameter("left_joint_name",     "left_wheel_joint"),
                           declare_parameter("right_joint_name",    "right_wheel_joint")};

        command_topics_ = {declare_parameter("left_command_topic",  "/motor/left/command"),
                           declare_parameter("right_command_topic", "/motor/right/command")};

        force_topics_   = {declare_parameter("left_force_topic",    "/motor/left/cmd_force"),
                           declare_parameter("right_force_topic",   "/motor/right/cmd_force")};

        // 两个电机子节点。joint_name / 话题 / control_mode 由底盘覆盖进去（这些是"这台车怎么
        // 接线"，底盘才知道），限幅 / 速度环 / 超时还是子节点自己的参数，params 文件按节点名给
        for (std::size_t i = 0; i < kWheelCount; ++i)
        {
            rclcpp::NodeOptions options;
            options.parameter_overrides({rclcpp::Parameter("joint_name",         joint_names_[i]),
                                                             rclcpp::Parameter("command_topic",       command_topics_[i]),
                                                             rclcpp::Parameter("force_topic",         force_topics_[i]),
                                                             rclcpp::Parameter("joint_states_topic",  topics_.joint_states_topic),
                                                             rclcpp::Parameter("control_mode",        control_.mode)});
            motor_[i] = std::make_shared<MotorNode>(kMotorNodeNames[i], options);
        }

        for (std::size_t i = 0; i < kWheelCount; ++i)
        {
            command_pub_[i] = create_publisher<std_msgs::msg::Float64>(command_topics_[i], 10);
        }

        // 回调只缓存最新一帧，控制律统一在定时器那一拍算：跟话题频率解耦，
        // 也不会因为 IMU 来了两帧、joint_states 只来了一帧就多算一次
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            topics_.imu_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            topics_.joint_states_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(*msg); });

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            remote_.cmd_vel_topic, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(*msg); });

        timer_ = create_wall_timer(std::chrono::duration<double>(control_.dt), [this]() { update(); });
    }

    /**
     * @brief 本节点和两个电机子节点，交给同一个 executor 一起 spin
     *
     * @return std::vector<rclcpp::Node::SharedPtr> 三个节点，第一个是底盘自己
     *
     * @note 子节点是裸 std::shared_ptr，不在 executor 里 spin 的话它们的订阅和定时器都不会跑 ——
     *       表现是"底盘在发指令，电机一点反应没有"。入口在 chassis.cpp
     */
    std::vector<rclcpp::Node::SharedPtr> nodes()
    {
        std::vector<rclcpp::Node::SharedPtr> all;
        all.reserve(kWheelCount + 1);
        all.push_back(shared_from_this());

        for (const auto& motor : motor_) {
            all.push_back(motor);
        }
        return all;
    }

private:
    // 两个电机子节点的节点名，顺序跟 kLeft / kRight 一致
    static constexpr const char* kMotorNodeNames[kWheelCount] = {"motor_left", "motor_right"};

    // 求解器。K 在构造时解一次，之后每拍只调 update() 算力矩
    Controller controller_;

    ModelParams   model_;
    ControlParams control_;
    RemoteParams  remote_;
    TopicParams   topics_;

    // 接线：哪个关节、指令发到哪个话题、力矩从哪个话题出去。三组都按下标对齐
    std::array<std::string, kWheelCount> joint_names_{};
    std::array<std::string, kWheelCount> command_topics_{};
    std::array<std::string, kWheelCount> force_topics_{};

    // 两个电机子节点，以及发给它们的指令（力矩 N·m）。
    // 裸 shared_ptr：不加进 executor 它们就不干活，见 nodes()
    std::array<MotorNode::SharedPtr, kWheelCount> motor_{};
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kWheelCount> command_pub_{};

    ChassisState state_;    // 最近一帧反馈，加上控制用的那四个量
    TeleopTarget teleop_;   // 遥控目标值
    StatusFlags  flags_;    // 反馈/标定/倒地，都是标志位

    // 最后一条 cmd_vel 的时刻。用 steady_clock：仿真时间会被暂停/复位
    std::chrono::steady_clock::time_point last_command_time_{};

    // ---- ROS 句柄 ----
    rclcpp::TimerBase::SharedPtr                                   timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;

    /**
     * @brief 参数 -> A/B -> K。整个生命周期只解这一次，解完打一行日志
     *
     * @note 方程怎么来的、这组参数解出来的 K 该是多少，见 docs/轮腿机器人LQR平衡教程.md §2
     *
     * @throw std::invalid_argument 参数不合法时抛（半径/步长非正、q 不是 4 个数）。
     *        这几个错都会让 K 变成 NaN，不拦的话表现是"车在抖"，比抛异常难查得多
     */
    void configure_lqr()
    {
        if (model_.wheel_radius <= 0.0 || control_.dt <= 0.0)
        {
            throw std::invalid_argument("wheel_radius 和 dt 必须大于 0");
        }
        if (control_.q_weights.size() != static_cast<std::size_t>(State::RowsAtCompileTime))
        {
            throw std::invalid_argument("参数 q 必须正好 4 个数：[位移, 速度, 俯仰, 俯仰角速度]");
        }

        // ---- 参数取短名字：下面两段就是教程 §2.1 的那几个公式，用 M/m/l/I/b 写才对得上 ----
        const double m = model_.body_mass;              // m，车身质量（除轮子外的全部）
        const double M = model_.cart_mass;              // M，轮子折合到轮轴上的质量
        const double l = model_.body_com_height;        // l，车身质心到轮轴的距离
        const double I = model_.body_pitch_inertia;     // I，车身绕质心的俯仰惯量
        const double b = model_.cart_damping;           // b，轮子那一路的等效粘性阻尼
        const double p = I * (M + m) + M * m * l * l;   // 共同分母，别约

        // ---- A：状态自己怎么变，跟控制无关 ----
        // 下标就是状态的顺序：0 位移、1 速度、2 俯仰、3 俯仰角速度。
        // 每一行读作"这个状态的导数等于谁"：
        //   A(0,1) = 1        位移的导数就是速度
        //   A(2,3) = 1        俯仰角的导数就是俯仰角速度
        //   第 1 行（速度的导数）     -(阻尼)·速度 - (重力)·俯仰
        //   第 3 行（俯仰角速度的导数）(阻尼)·速度 + (重力)·俯仰
        // 关键在 A(3,2) 是**正**的：俯仰一歪，重力就让车身越歪越快。倒立摆"站不住"就是
        // 这一项来的（教程 §2.1 里那个 +13.55 的特征值），所以要靠下面的 K 把它拉回来
        StateMatrix continuous_a = StateMatrix::Zero();
        continuous_a(0, 1) =  1.0;
        continuous_a(1, 1) = -(I + m * l * l) * b / p;
        continuous_a(1, 2) = -m * m * model_.gravity * l * l / p;
        continuous_a(2, 3) =  1.0;
        continuous_a(3, 1) =  m * l * b / p;
        continuous_a(3, 2) =  m * model_.gravity * l * (M + m) / p;

        // ---- B：控制从哪里进来。力矩只作用在轮子上，所以只有两行有值 ----
        // 给正力矩 = 轮子往前滚 = 车往前走（第 1 行），同时车身被反作用顶得往后仰（第 3 行，
        // 所以那个数是负的）。这两行就是"推车"和"把车推倒"的竞争关系
        // B 里的 2.0：u 是"两个轮子各给一份"的单轮力矩，整车得到的推力是 2u。
        // 这么写的好处是解出来的 K 直接就是单轮力矩，用时不用再除 2 或者乘 2
        const double body_pitch_about_com = I + m * l * l;   // 车身绕自己质心的俯仰惯量
        InputMatrix continuous_b = InputMatrix::Zero();
        continuous_b(1, 0) =  2.0 * (body_pitch_about_com / model_.wheel_radius + m * l) / p;
        continuous_b(3, 0) = -2.0 * (m * l / model_.wheel_radius + M + m) / p;

        // ---- 上面是连续时间的方程，但控制器是每 dt 才算一次的，得先离散化 ----
        // 前向欧拉：A_d = I + A·dt、B_d = B·dt（教程 §2.2）。dt = 1 ms，再高的精度在这个步长上没有意义
        const StateMatrix discrete_a = StateMatrix::Identity() + continuous_a * control_.dt;
        const InputMatrix discrete_b = continuous_b * control_.dt;

        // ---- Q：哪个状态偏了罚得重。对角线上第 i 个数就是第 i 个状态的罚分 ----
        // 顺序还是 位移/速度/俯仰/俯仰角速度。只有比值有意义，绝对值无所谓：
        // 这里俯仰 1000、俯仰角速度 550，位移和速度各 1 —— "宁可位置跑偏，也不能倒"。
        // 四个数都别给 0：§2.1 里 0 附近那对特征值（-0.0106 / 0）是"车身直立、车匀速跑"
        // 这条中性方向，不罚它，就没有哪一项会把它拉回来
        StateMatrix q = StateMatrix::Zero();
        for (std::size_t i = 0; i < control_.q_weights.size(); ++i)
        {
            q(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) = control_.q_weights[i];
        }

        // ---- R：用力矩本身也要罚。给得越大，K 越小、动作越柔和，但站得也越软 ----
        ControlMatrix r = ControlMatrix::Zero();
        r(0, 0) = control_.r_weight;

        // ---- 解离散黎卡提方程，拿最优增益 K ----
        // 这一步只跟 A/B/Q/R 有关、跟车当前的状态无关，所以整个生命周期只做一次；
        // 之后每拍只是拿 K 做一次矩阵乘法（见 update()）
        controller_.configure(discrete_a, discrete_b, q, r);
        controller_.solve(100000, 1e-12);   // 迭代到收敛：1e-12 是"相邻两次 P 的差"，不是误差上限

        // K 就是控制律 u = -K(x - target) 里那个 K：一行四列，一列对应一个状态
        const auto& k = controller_.gain();
        RCLCPP_INFO(get_logger(), "LQR K = [%.6f, %.6f, %.6f, %.6f]", k(0, 0), k(0, 1), k(0, 2), k(0, 3));
    }

    /**
     * @brief 收到一帧 IMU：取出俯仰角和俯仰角速度
     *
     * @param msg 原始 IMU。只用 orientation 和 angular_velocity，加速度计不参与
     *
     * @note 仿真里 orientation 是位姿真值（gz 直接算的，不是陀螺积分），所以不漂、直接转
     *       欧拉角就行。实车上这一处要换成 imu 节点的融合输出，见教程 §3.3
     */
    void on_imu(const sensor_msgs::msg::Imu& msg)
    {
        const double w = msg.orientation.w;
        const double x = msg.orientation.x;
        const double y = msg.orientation.y;
        const double z = msg.orientation.z;

        // 绕 +y 的俯仰角（ZYX 欧拉角里的 pitch 分量）。clamp 是防浮点越界让 asin 出 NaN
        const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
        state_.pitch      = control_.pitch_sign * std::asin(sin_pitch);
        state_.pitch_rate = control_.pitch_sign * msg.angular_velocity.y;
        state_.yaw_rate   = msg.angular_velocity.z;
        flags_.has_imu    = true;
    }

    /**
     * @brief 收到一帧关节状态：挑出两个轮关节，折成整车的位移和速度
     *
     * @param msg 关节状态。这一帧里必须两个轮子都在
     *
     * @note position 是**累积角**（continuous 关节一直涨），所以这里只是"相对原点的里程"，
     *       原点在 update() 里标定。忘了减原点的话，车一起步就以为自己在很远的地方
     */
    void on_joint_state(const sensor_msgs::msg::JointState& msg)
    {
        // name / position / velocity 三个数组不保证一样长，一起截短，别各自判越界
        const std::size_t count = std::min({msg.name.size(), msg.position.size(), msg.velocity.size()});

        double      position_sum = 0.0;
        double      velocity_sum = 0.0;
        std::size_t wheel_count  = 0;

        for (std::size_t i = 0; i < count; ++i)
        {
            if (msg.name[i] != joint_names_[kLeft] && msg.name[i] != joint_names_[kRight])
            {
                continue;
            }
            position_sum += msg.position[i];
            velocity_sum += msg.velocity[i];
            ++wheel_count;
        }

        // 两个轮子都在这一帧里才认。只报一个说明这帧不完整，宁可沿用上一帧，
        // 也别拿"半个轮子"的状态当整车状态
        if (wheel_count != kWheelCount)
        {
            return;
        }

        // 取两个轮子的平均：一起往前走时各贡献一半，差的那一半是偏航，不算进位移
        state_.wheel_position  = model_.wheel_radius * position_sum * 0.5;
        state_.wheel_velocity  = model_.wheel_radius * velocity_sum * 0.5;
        flags_.has_joint_state = true;
    }

    /**
     * @brief 收到一条遥控指令：限幅之后当成目标，别的一概不管
     *
     * @param msg Twist。只读 linear.x（m/s，正 = 前进）和 angular.z（rad/s，正 = 左转）
     *
     * @note linear.x 先乘 linear_sign 再限幅：这套符号只作用在**目标值**上，
     *       状态（轮子/俯仰）和力矩那两路不动，平衡环自洽，翻它没用
     * @note 先用 isfinite 挡一遍再 clamp：std::clamp(NaN, ...) 不保证挡得住，一个 NaN
     *       进来整个状态就是 NaN，表现是车突然不动了或者抽一下
     */
    void on_cmd_vel(const geometry_msgs::msg::Twist& msg)
    {
        if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.angular.z))
        {
            return;   // 丢掉这一条，留着上一条；之后超时了照样会停车
        }

        teleop_.velocity = std::clamp(remote_.linear_sign * msg.linear.x, -remote_.max_velocity, remote_.max_velocity);
        teleop_.yaw_rate = std::clamp(msg.angular.z, -remote_.max_yaw_rate, remote_.max_yaw_rate);

        flags_.has_command = true;
        last_command_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 最新一条指令到现在过了多久
     *
     * @return double 秒；一条指令都没收到过时返回无穷大，也就是"一直不新鲜"
     *
     * @note 用 steady_clock 而不是仿真时间：仿真会被暂停/复位，拿它算"多久没收到指令"会判错
     */
    double command_age_s() const
    {
        if (!flags_.has_command)
        {
            return std::numeric_limits<double>::infinity();
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_time_).count();
    }

    /**
     * @brief 控制节拍：把最新一帧反馈喂给 LQR，把力矩发给两个轮子
     *
     * @note 回调只缓存，整形和求解都在这一拍里做，跟话题频率解耦
     */
    void update()
    {
        // 反馈没齐（IMU 或关节状态一帧都没到）就不发力：拿 0 当状态等于盲发力
        if (!flags_.has_imu || !flags_.has_joint_state)
        {
            publish_effort(0.0, 0.0);
            return;
        }

        // 姿态超限 = 已经倒了。归零不是"保护电机"，是"别躺着还使劲蹬地"
        if (std::abs(state_.pitch) > control_.fall_angle_rad)
        {
            flags_.fallen = true;
            publish_effort(0.0, 0.0);
            return;
        }

        // 刚被扶正：倒地期间轮子在空转，里程已经飞了，重新标定原点再接手
        if (flags_.fallen)
        {
            flags_.fallen             = false;
            flags_.origin_initialized = false;
            state_.wheel_velocity     = 0.0;
            teleop_.position          = 0.0;
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

        // 位置目标由速度指令积分而来，速度目标直接给。让 LQR 追速度、而不是死盯一个位移点
        // —— 死盯位移的话，车为了"停在原地"会顶着速度指令不动
        teleop_.position += teleop_.velocity * control_.dt;

        State target = State::Zero();
        target(0) = teleop_.position;
        target(1) = teleop_.velocity;

        State state;
        state << state_.wheel_position - state_.position_origin, state_.wheel_velocity, state_.pitch, state_.pitch_rate;

        // 公共力矩：俯仰 + 前后走都算在这一份里，夹到 max_effort 是它的预算
        const double effort = std::clamp(control_.control_sign * controller_.update(state, target)(0), -control_.max_effort, control_.max_effort);

        // 转向：左右轮反向偏置，力矩差产生偏航力矩（正 = 左转）。反馈只有角速度
        // （IMU 的 angular_velocity.z），没有偏航角，所以这条环管的是"转多快"，不是"转到哪"
        const double differential = std::clamp(remote_.yaw_sign * remote_.yaw_kp * (teleop_.yaw_rate - state_.yaw_rate),
                                               -remote_.max_differential, remote_.max_differential);

        publish_effort(effort - differential, effort + differential);
    }

    /**
     * @brief 把两个轮子的力矩发出去
     *
     * @param left_effort 左轮力矩 N·m
     * @param right_effort 右轮力矩 N·m
     *
     * @note 单轮上限多了 max_differential 那一份余量：公共力矩最多 max_effort、转向差分最多
     *       max_differential，所以单轮最大是两者之和。只夹到 max_effort 的话，公共力矩一贴轨，
     *       差分正方向那一侧就被压平 —— 表现是"直着能站住，一转弯就不听使唤"
     */
    void publish_effort(double left_effort, double right_effort)
    {
        const double wheel_limit = control_.max_effort + remote_.max_differential;

        std_msgs::msg::Float64 left;
        std_msgs::msg::Float64 right;
        left.data  = std::clamp(left_effort,  -wheel_limit, wheel_limit);
        right.data = std::clamp(right_effort, -wheel_limit, wheel_limit);

        command_pub_[kLeft]->publish(left);
        command_pub_[kRight]->publish(right);
    }
};
