/**
 * @file chassis_config.hpp
 * @brief 集中声明、读取并校验 ChassisNode 的 ROS 参数
 *
 * @note 全节点只有这里碰参数服务器：构造函数调一次 declare_from()，之后各模块拿到的都是
 *       已经过 validate() 的结构体，运行期不再回读参数（除了消融开关由外部 ros2 param set 改）
 * @note 参数名跟 params/chassis.yaml 里的键一一对应；默认值只是"文件读不到时的兜底"，
 *       真正的数值以 yaml 为准，改默认值不等于改行为
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "chassis_types.hpp"
#include "leg_height.hpp"
#include "torque_allocator.hpp"

/**
 * @brief 节点跑起来要的全部配置：各参数组 + 接线 + 增益文件路径 + 模型描述
 *
 * @note 这里面没有任何运行期状态：状态在 ChassisState / 各模块自己手里，这份是只读的
 */
struct ChassisConfiguration
{
    ModelParams   model;                                    // 模型参数：由 URDF 量出来，离线生成器也读同一份
    ControlParams control;                                  // 节拍、共模力矩上限、符号、单轮力矩变化率
    RemoteParams  remote;                                   // 遥控：cmd_vel、速度/偏航上限、身高步长、指令超时
    TopicParams   topics;                                   // 反馈话题（IMU / 关节状态）
    LegParams     leg;                                      // 四个腿关节：接线、限位、软限位、变化率
    SafetyParams  safety;                                   // 反馈超时、节拍卡顿、故障恢复驻留
    HeightParams  height;                                   // 目标腿高、变化率、换算与重力前馈要的几何/质量
    ActuatorEnableParams       enabled;                     // 五路消融开关，默认全开
    std::array<std::string, 2> wheel_joint_names {};        // 两个轮关节名，顺序 [左, 右]
    std::array<std::string, 2> wheel_command_topics {};     // 两条轮指令话题，顺序 [左, 右]
    std::vector<std::string>   gain_paths;                  // 增益文件绝对路径，按腿高升序（调度用）
    std::string                robot_description;           // 参数传进来的 URDF 文本，用来核对增益文件的模型哈希

    /**
     * @brief 一次性建立完整配置：声明并读取全部参数，校验通过后返回
     *
     * @param node 用哪个节点声明参数；参数名就是 params/chassis.yaml 里那些键
     * @param gain_directory 增益文件所在目录的绝对路径；gain_files 里只写文件名（不带 '/'）
     *                       的会被拼上它
     * @return ChassisConfiguration 校验过的配置；读不到或者不合法就直接抛，不会返回半套
     *
     * @throw std::invalid_argument 配置校验不过（见 validate()）
     * @throw std::runtime_error 参数缺失或类型不对时由 rclcpp 抛出
     *
     * @note 每个节点只能调一次：declare_parameter 对同名参数重复声明会抛
     */
    static ChassisConfiguration declare_from(rclcpp::Node& node, const std::string& gain_directory)
    {
        ChassisConfiguration result;

        // ---- 模型参数 ----
        auto& model = result.model;
        model.wheel_radius       = node.declare_parameter("wheel_radius",       model.wheel_radius);
        model.cart_mass          = node.declare_parameter("cart_mass",          model.cart_mass);
        model.body_mass          = node.declare_parameter("body_mass",          model.body_mass);
        model.body_com_height    = node.declare_parameter("body_com_height",    model.body_com_height);
        model.body_pitch_inertia = node.declare_parameter("body_pitch_inertia", model.body_pitch_inertia);
        model.cart_damping       = node.declare_parameter("cart_damping",       model.cart_damping);
        model.gravity            = node.declare_parameter("gravity",            model.gravity);

        // ---- 控制：上限和角度取绝对值，符号单独留（pitch_sign / control_sign 允许为负）----
        auto& control = result.control;
        control.dt              = node.declare_parameter("dt",              control.dt);
        control.max_effort      = std::abs(node.declare_parameter("max_effort",     control.max_effort));
        control.fall_angle_rad  = std::abs(node.declare_parameter("fall_angle_rad", control.fall_angle_rad));
        control.pitch_sign      = node.declare_parameter("pitch_sign",      control.pitch_sign);
        control.control_sign    = node.declare_parameter("control_sign",    control.control_sign);
        control.mode            = node.declare_parameter("control_mode",    control.mode);
        control.wheel_effort_rate_limit = std::abs(node.declare_parameter("wheel_effort_rate_limit", control.wheel_effort_rate_limit));
        
        // 这些权重只供离线生成器读同一份 YAML；运行时声明后丢弃，不把它们混进控制器状态。
        (void)node.declare_parameter<std::vector<double>>("q_pitch", {700.0, 200.0, 1000.0, 550.0, 300.0, 30.0, 300.0, 30.0});
        (void)node.declare_parameter<std::vector<double>>("r_pitch", {1.0,   2.0,   2.0});
        (void)node.declare_parameter<std::vector<double>>("q_roll",  {800.0, 60.0,  400.0,  40.0,  400.0, 40.0});
        (void)node.declare_parameter<std::vector<double>>("r_roll",  {2.0,   2.0});
 
        // ---- 遥控：速度/偏航上限、身高步长、指令超时 ----
        auto& remote = result.remote;

        remote.cmd_vel_topic = node.declare_parameter("cmd_vel_topic",           remote.cmd_vel_topic);
        remote.max_velocity  = std::abs(node.declare_parameter("max_velocity", remote.max_velocity));
        remote.max_yaw_rate  = std::abs(node.declare_parameter("max_yaw_rate", remote.max_yaw_rate));
        remote.height_step_m = node.declare_parameter("height_step_m",           remote.height_step_m);
        remote.yaw_kp        = std::abs(node.declare_parameter("yaw_kp",      remote.yaw_kp));
        remote.max_differential  = std::abs(node.declare_parameter("max_differential", remote.max_differential));
        remote.command_timeout_s = std::abs(node.declare_parameter("command_timeout_s", remote.command_timeout_s));

        // ---- 安全监督：反馈超时、节拍卡顿、恢复驻留 ----
        auto& safety = result.safety;
        safety.feedback_timeout_s   = std::abs(node.declare_parameter("feedback_timeout_s",   safety.feedback_timeout_s));
        safety.max_control_period_s = std::abs(node.declare_parameter("max_control_period_s", safety.max_control_period_s));
        safety.recovery_dwell_s     = std::abs(node.declare_parameter("recovery_dwell_s",     safety.recovery_dwell_s));

        // ---- 消融开关：五路，运行中可以用 ros2 param set 改，用来复现"少了某一路会怎样" ----
        auto& enabled = result.enabled;
        enabled.wheel_common = node.declare_parameter("enable_wheel_common", enabled.wheel_common);
        enabled.hip_common   = node.declare_parameter("enable_hip_common",   enabled.hip_common);
        enabled.knee_common  = node.declare_parameter("enable_knee_common",  enabled.knee_common);
        enabled.hip_diff     = node.declare_parameter("enable_hip_diff",     enabled.hip_diff);
        enabled.knee_diff    = node.declare_parameter("enable_knee_diff",    enabled.knee_diff);

        // ---- 反馈话题 ----
        auto& topics = result.topics;
        topics.imu_topic          = node.declare_parameter("imu_topic", topics.imu_topic);
        topics.joint_states_topic = node.declare_parameter("joint_states_topic", topics.joint_states_topic);

        // ---- 增益文件：只给文件名就当在本包 gains/ 目录里，带斜杠的按原样用 ----
        const std::string gain_file = node.declare_parameter("gain_file", gain_directory + "nominal.yaml");
        auto gain_files = node.declare_parameter<std::vector<std::string>>("gain_files", 
            {"height_058.yaml", "nominal.yaml", "height_080.yaml", "height_090.yaml", "height_100.yaml", "height_104.yaml"});
        if (gain_files.empty())
        {
            gain_files.push_back(gain_file);
        }
        for (const auto& configured_path : gain_files)
        {
            result.gain_paths.push_back(configured_path.find('/') == std::string::npos ? gain_directory + configured_path : configured_path);
        }
        // URDF 文本从参数来（launch 把 robot_description 一起喂进来），用来核对增益文件里的模型哈希
        result.robot_description = node.declare_parameter("robot_description", std::string{});

        // ---- 两个轮子的接线，顺序 [左, 右] ----
        result.wheel_joint_names    = {node.declare_parameter("left_joint_name",     "left_wheel_joint"),
                                       node.declare_parameter("right_joint_name",    "right_wheel_joint")};
        result.wheel_command_topics = {node.declare_parameter("left_command_topic",  "/motor/left/command"),
                                       node.declare_parameter("right_command_topic", "/motor/right/command")};

        // ---- 四个腿关节：接线、限位、软限位、变化率，顺序 [左髋, 右髋, 左膝, 右膝] ----
        auto& leg = result.leg;
        leg.joint_names     = node.declare_parameter<std::vector<std::string>>("leg_joint_names",    leg.joint_names);
        leg.command_topics  = node.declare_parameter<std::vector<std::string>>("leg_command_topics", leg.command_topics);
        leg.max_effort      = node.declare_parameter<std::vector<double>>("leg_max_effort",          leg.max_effort);
        leg.max_speed_radps = node.declare_parameter<std::vector<double>>("leg_max_speed_radps",     leg.max_speed_radps);
        leg.limit_lower     = node.declare_parameter<std::vector<double>>("leg_limit_lower",         leg.limit_lower);
        leg.limit_upper     = node.declare_parameter<std::vector<double>>("leg_limit_upper",         leg.limit_upper);
        leg.joint_sign      = node.declare_parameter<std::vector<double>>("leg_joint_sign",          leg.joint_sign);
        leg.soft_limit_kp   = node.declare_parameter<std::vector<double>>("leg_soft_limit_kp",       leg.soft_limit_kp);
        leg.soft_limit_kd   = node.declare_parameter<std::vector<double>>("leg_soft_limit_kd",       leg.soft_limit_kd);
        leg.effort_rate_limit = node.declare_parameter<std::vector<double>>("leg_effort_rate_limit", leg.effort_rate_limit);
        leg.soft_limit_margin = node.declare_parameter<std::vector<double>>("leg_soft_limit_margin", leg.soft_limit_margin);

        // ---- 腿高：杆长/夹角是几何（跟 modules/leg.xacro 一份数），质量用来随高度重算重力前馈 ----
        auto& height = result.height;
        height.link_length      = node.declare_parameter("leg_length",      height.link_length);
        height.link_angle       = node.declare_parameter("leg_angle",       height.link_angle);
        height.target_height_m  = node.declare_parameter("target_height_m", height.target_height_m);
        height.maximum_rate_mps = node.declare_parameter("height_rate_mps", height.maximum_rate_mps);
        height.rod_mass         = node.declare_parameter("leg_rod_mass",    height.rod_mass);
        height.joint_mass       = node.declare_parameter("leg_joint_mass",  height.joint_mass);
        height.wheel_mass       = node.declare_parameter("wheel_mass",      height.wheel_mass);

        result.validate();
        return result;
    }

    /**
     * @brief 校验读进来的配置：错就抛，别让半套参数跑到控制律里
     *
     * @throw std::invalid_argument 模式不对、节拍/几何非正、腿数组长度不对、限位或软限位矛盾时抛
     *
     * @note 软限位那条 2*margin < upper-lower 是防呆：margin 太大会让两个软限位区间交叉，
     *       恢复力矩在中间就开始互相顶
     */
    void validate() const
    {
        if (control.mode != "effort")
        {
            throw std::invalid_argument("平衡环必须用 effort 模式");
        }
        if (!std::isfinite(control.dt) || control.dt <= 0.0 ||
            !std::isfinite(model.wheel_radius) || model.wheel_radius <= 0.0)
        {
            throw std::invalid_argument("dt 和 wheel_radius 必须是有限正数");
        }

        // 腿的参数是按 [左髋, 右髋, 左膝, 右膝] 对齐的数组，长度不齐后面全是错位
        constexpr std::size_t count = 4;
        if (leg.joint_names.size()       != count || leg.command_topics.size()      != count ||
            leg.max_effort.size()        != count || leg.max_speed_radps.size()     != count ||
            leg.limit_lower.size()       != count || leg.limit_upper.size()         != count ||
            leg.joint_sign.size()        != count || leg.effort_rate_limit.size()   != count ||
            leg.soft_limit_margin.size() != count || leg.soft_limit_kp.size()       != count ||
            leg.soft_limit_kd.size()     != count)
        {
            throw std::invalid_argument("腿参数数组必须都是 4 项");
        }
        for (std::size_t i = 0; i < count; ++i)
        {
            if (leg.limit_lower[i] >= leg.limit_upper[i] || leg.max_effort[i] <= 0.0 || leg.effort_rate_limit[i] <= 0.0 || leg.soft_limit_margin[i] <= 0.0 ||
                2.0 * leg.soft_limit_margin[i] >= leg.limit_upper[i] - leg.limit_lower[i] || leg.soft_limit_kp[i] < 0.0 || leg.soft_limit_kd[i] < 0.0)
            {
                throw std::invalid_argument("腿关节力矩、变化率或软硬限位参数无效：" + leg.joint_names[i]);
            }
        }

        // 变化率、时间常量和身高步长：都得是正的；节拍上限必须大于正常周期，否则一开机就判卡顿
        if (control.wheel_effort_rate_limit <= 0.0 || safety.feedback_timeout_s <= 0.0 || safety.max_control_period_s <= control.dt || 
            safety.recovery_dwell_s < 0.0 || !std::isfinite(remote.height_step_m) || remote.height_step_m <= 0.0)
        {
            throw std::invalid_argument("底盘变化率、时间或身高步长参数无效");
        }
        if (!std::isfinite(height.link_length) || height.link_length <= 0.0 || !std::isfinite(height.link_angle) || !std::isfinite(height.maximum_rate_mps) || 
                              height.maximum_rate_mps <= 0.0 || height.rod_mass <= 0.0 || height.joint_mass <= 0.0 || height.wheel_mass <= 0.0)
        {
            throw std::invalid_argument("腿高几何、质量和变化率必须为有限正数");
        }
    }

    /**
     * @brief 把配置折成力矩分配器要的那份：六路顺序 [左轮, 右轮, 左髋, 右髋, 左膝, 右膝]
     *
     * @note 单轮上限是 max_effort + max_differential：只夹到 max_effort 的话，公共力矩一贴轨，
     *       偏航那一侧的差动就被压平 —— 直着能站住，一转弯就不听使唤
     */
    TorqueAllocator::Config allocator_config() const
    {
        TorqueAllocator::Config result;
        const double wheel_limit = control.max_effort + remote.max_differential;

        result.dt = control.dt;
        result.effort_limit      = {wheel_limit, wheel_limit, leg.max_effort[0], leg.max_effort[1], leg.max_effort[2], leg.max_effort[3]};
        result.effort_rate_limit = {control.wheel_effort_rate_limit, control.wheel_effort_rate_limit, leg.effort_rate_limit[0], 
                                    leg.effort_rate_limit[1],        leg.effort_rate_limit[2],        leg.effort_rate_limit[3]};

        for (std::size_t i = 0; i < TorqueAllocator::kLegCount; ++i)
        {
            result.hard_lower[i] = leg.limit_lower[i];
            result.hard_upper[i] = leg.limit_upper[i];

            // 软限位 = 硬限位往里收 margin；恢复力矩在软硬之间那段起作用
            result.soft_lower[i] = leg.limit_lower[i] + leg.soft_limit_margin[i];
            result.soft_upper[i] = leg.limit_upper[i] - leg.soft_limit_margin[i];
            result.soft_limit_kp[i] = leg.soft_limit_kp[i];
            result.soft_limit_kd[i] = leg.soft_limit_kd[i];
        }
        return result;
    }

    /**
     * @brief 腿高换算要的几何：杆长、夹角，以及软限位内的髋膝可达区间
     *
     * @note 髋/膝的区间左右各取一次交集（取更紧的那一侧）：腿高是共模量，任何一侧先碰到
     *       软限位，整条伸缩就得停在那儿，所以能用的是两侧的交集
     */
    LegHeightKinematics::Config height_kinematics_config() const
    {
        LegHeightKinematics::Config result;
        
        result.hip_lower          = std::max(leg.limit_lower[0] + leg.soft_limit_margin[0], leg.limit_lower[1] + leg.soft_limit_margin[1]);
        result.hip_upper          = std::min(leg.limit_upper[0] - leg.soft_limit_margin[0], leg.limit_upper[1] - leg.soft_limit_margin[1]);
        result.knee_lower         = std::max(leg.limit_lower[2] + leg.soft_limit_margin[2], leg.limit_lower[3] + leg.soft_limit_margin[3]);
        result.knee_upper         = std::min(leg.limit_upper[2] - leg.soft_limit_margin[2], leg.limit_upper[3] - leg.soft_limit_margin[3]);
        result.link_length        = height.link_length;
        result.nominal_link_angle = height.link_angle;

        return result;
    }
};
