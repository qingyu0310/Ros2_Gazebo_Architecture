/**
 * @file chassis_types.hpp
 * @brief 底盘节点、配置加载器和安全监督器共享的数据类型
 *
 * @note 这里只放数据结构，不带任何逻辑：默认值就是"参数文件读不到时的兜底"，参数文件里
 *       同名键的值会覆盖它（长度检查、符号处理那些校验在 chassis_config.hpp）
 * @note 腿关节的数组一律按 [左髋, 右髋, 左膝, 右膝] 排列，六个执行器按
 *       [左轮, 右轮, 左髋, 右髋, 左膝, 右膝] 排列 —— 顺序是接口的一部分，别在这里改
 */

#pragma once

#include <array>
#include <string>
#include <vector>

/**
 * @brief 模型参数：由 URDF 量出来，改模型就要重算（对不上不报错，只是站不住）
 */
struct ModelParams
{
    double wheel_radius       {0.030};          // r，跟 modules/wheel.xacro 的 radius 对齐
    double cart_mass          {0.9000};         // M = 2*(轮子质量 + 轮子自转惯量/r²)；自转惯量是模型里的 izz
    double body_mass          {1.25};           // m，除两个轮子外的整车质量
    double body_com_height    {0.06825};        // l，车身质心到轮轴的距离
    double body_pitch_inertia {0.002591};       // I，车身绕质心、绕 y 轴的俯仰惯量
    double cart_damping       {0.02};           // b，等效粘性阻尼，估的量（0~0.05 都行）
    double gravity            {9.81};
};

/**
 * @brief 控制参数：节拍、共模力矩上限、符号、单轮力矩变化率
 */
struct ControlParams
{
    double dt               {0.001};            // 控制周期 s，要跟 gz 物理步长、定时器实际周期一致
    double max_effort       {0.25};             // LQR 公共力矩上限 N·m（参数文件里给 0.20，附着极限约 0.29）
    double fall_angle_rad   {0.70};             // |俯仰| 超过它（40°）就认定倒地，力矩归零
    double pitch_sign       {1.0};              // 俯仰方向反了翻成 -1.0
    double control_sign     {1.0};              // 力矩方向反了翻成 -1.0
    double wheel_effort_rate_limit {100.0};     // 单轮力矩变化率上限 N·m/s（1 ms 一拍最多变 0.10 N·m）
    std::string mode        {"effort"};      // 只能是 effort：中间插一层速度环会和平衡环对着干
};

/**
 * @brief 腿关节参数：四个腿关节（顺序 [左髋, 右髋, 左膝, 右膝]）
 *
 * @note 上下限抄的是模型里的硬限位（髋 -15°~35°、膝 -65°~25°）；软限位从硬限位向内收
 *       margin，给恢复力矩留空间，别把两者写成一个数
 */
struct LegParams
{
    std::vector<std::string> joint_names    {"left_hip_joint",          "right_hip_joint",          "left_knee_joint",          "right_knee_joint"};
    std::vector<std::string> command_topics {"/motor/left_hip/command", "/motor/right_hip/command", "/motor/left_knee/command", "/motor/right_knee/command"};
    std::vector<double> max_effort          {1.5,       1.5,        1.5,        1.5};        // 单关节力矩上限 N·m，模型里的硬限位
    std::vector<double> max_speed_radps     {6.0,       6.0,        6.0,        6.0};        // 关节转速上限 rad/s，力矩模式下只是护栏
    std::vector<double> limit_lower         {-0.2618,   -0.2618,    -1.1345,    -1.1345};    // 硬限位下限 rad（髋 -15°、膝 -65°）
    std::vector<double> limit_upper         {0.6109,    0.6109,     0.4363,     0.4363};     // 硬限位上限 rad（髋 +35°、膝 +25°）
    std::vector<double> joint_sign          {1.0,       1.0,        1.0,        1.0};        // 正方向反了翻成 -1.0
    std::vector<double> effort_rate_limit   {500.0,     500.0,      500.0,      500.0};      // 力矩变化率上限 N·m/s（1 ms 一拍最多 0.5 N·m）
    std::vector<double> soft_limit_margin   {0.10,      0.10,       0.10,       0.10};       // 软限位相对硬限位向内收 rad
    std::vector<double> soft_limit_kp       {2.0,       2.0,        2.0,        2.0};        // 越过软限位后的恢复刚度 N·m/rad
    std::vector<double> soft_limit_kd       {0.05,      0.05,       0.05,       0.05};       // 恢复阻尼 N·m·s/rad
};

/**
 * @brief 安全监督参数：反馈超时、节拍卡顿、故障恢复
 */
struct SafetyParams
{
    double feedback_timeout_s   {0.10};  // IMU/关节反馈超过这么久没更新就判故障，六路清零
    double max_control_period_s {0.02};  // 相邻两拍的控制周期超过它就判卡顿（1 ms 的 20 倍）
    double recovery_dwell_s     {0.08};  // 故障恢复要连续健康这么久才重新接管控制
};

/**
 * @brief 执行器消融开关：默认全开，关掉某一路用来验证"少了它性能怎么退"
 *
 * @note 共模那三路是 Pitch 的执行器，差模那两路是 Roll 的；关掉之后的闭环特征值应该在
 *       增益文件的 ablation 里能查到预测值，实测该跟它对得上
 */
struct ActuatorEnableParams
{
    bool wheel_common {true};
    bool hip_common   {true};
    bool knee_common  {true};
    bool hip_diff     {true};
    bool knee_diff    {true};
};

/**
 * @brief 腿高参数：目标腿高、变化率，以及换算与重力前馈要的几何/质量
 *
 * @note 对称伸缩时腿高 h = 2L·sin(α0 + q_h)，目标腿高的可达范围由软限位算出来（启动时打印）
 */
struct HeightParams
{
    double link_length      {0.056};            // L，单根大腿/小腿长度 m
    double link_angle       {0.6981317008};     // α0，零关节角时杆与水平面的夹角 rad（40°）
    double target_height_m  {0.0719922123};     // 目标腿高：髋轴到轮轴的竖直距离 m，标准姿态 = 2*leg_drop
    double maximum_rate_mps {0.01};             // 目标腿高最大变化速度 m/s；把阶跃变成缓坡，避免冲击平衡环
    double rod_mass         {0.03};             // 单根腿杆质量 kg，随高度重算静态重力前馈用
    double joint_mass       {0.005};            // 单个膝部关节块质量 kg
    double wheel_mass       {0.15};             // 单轮质量 kg
};

/**
 * @brief 遥控接口参数：键盘发 Twist 的地方，以及归一化指令对应的实际上限
 */
struct RemoteParams
{
    std::string cmd_vel_topic {"/chassis/cmd_vel"};  // 键盘往这里发 Twist（linear.x / angular.z 取 ±1）
    double max_velocity       {0.20};           // 前进速度上限 m/s（±1 指令对应它）
    double max_yaw_rate       {1.0};            // 偏航角速度上限 rad/s
    double height_step_m      {0.001};          // 每收到一次 '=' / '-' 脉冲，目标腿高增减 1 mm
    double yaw_kp             {0.01};           // 偏航角速度误差 -> 力矩差，N·m·s/rad
    double max_differential   {0.02};           // 力矩差上限 N·m（单轮上限 = 它 + max_effort）
    double command_timeout_s  {0.30};           // 这么久没收到 cmd_vel 就当停车，断了不会一直冲
};

/**
 * @brief 反馈话题：要跟模型里传感器宏的话题对上
 */
struct TopicParams
{
    std::string imu_topic {"/imu"};
    std::string joint_states_topic {"/joint_states"};
};

/**
 * @brief 底盘状态：最近一帧反馈里挑出来的量
 *
 * @note roll/roll_rate 只用于安全检查和闭链一致性诊断 —— Roll 子系统里没有独立的 roll 状态
 *       （两轮贴地时 roll 由左右腿高差唯一决定），LQR 吃的是四个腿差模量
 */
struct ChassisState
{
    double roll             {0.0};                  // 机身横滚 rad（IMU）
    double roll_rate        {0.0};                  // 横滚角速度 rad/s（IMU 陀螺）
    double pitch            {0.0};                  // 机身俯仰 rad，前倾为正
    double pitch_rate       {0.0};                  // 俯仰角速度 rad/s
    double yaw_rate         {0.0};                  // 偏航角速度 rad/s，只给偏航外环用
    double wheel_position   {0.0};                  // r*（左轮角+右轮角）/2，轮子转过的角折成"走了多远" m
    double wheel_velocity   {0.0};                  // 同上，速度 m/s
    double position_origin  {0.0};                  // 位移原点，减掉它才是里程（倒地扶正/重新接管时重标）
    std::array<double, 4> leg_position_rad   {};    // 四个腿关节角 rad，顺序同 LegParams
    std::array<double, 4> leg_velocity_radps {};    // 四个腿关节角速度 rad/s
    std::array<bool, 4> leg_valid            {};    // 这一帧里有没有它；false 时上面两个值不可信
};

/**
 * @brief 遥控目标：cmd_vel 解出来的两个目标值，外加由速度积分出来的位置目标
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
 * @brief 标志位：反馈来没来、原点标定没标定
 *
 * @note 反馈标志必须先置上才发力：拿 0 当状态等于盲发力
 */
struct StatusFlags
{
    bool has_imu            {false};    // 收到过 IMU 没有
    bool has_joint_state    {false};    // 收到过"两轮都在"的一帧没有
    bool origin_initialized {false};    // 位移原点标定过没有
    bool has_command        {false};    // 收到过 cmd_vel 没有
};
