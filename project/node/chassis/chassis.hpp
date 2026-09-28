/**
 * @file chassis.hpp
 * @author qingyu
 * @brief 底盘 ROS 编排节点：汇集反馈、调用控制模块并发布六路力矩
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 * @note 参数加载、增益调度、安全监督、Pitch/Roll LQR、腿高与力矩分配均已拆成独立类。
 *       本节点只负责 ROS 接线、反馈缓存和一拍控制流程的编排。
 * @note 话题：订 /imu、/joint_states、/chassis/cmd_vel；发六路 /motor/<执行器>/command（N·m）
 * @note K/X0/U0 从离线生成的 params/leg_gain/nominal.yaml 加载；接口或模型不一致时拒绝启动
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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "chassis_config.hpp"
#include "chassis_types.hpp"
#include "leg_height.hpp"
#include "leg_gravity_compensation.hpp"
#include "lqr_gain_schedule.hpp"
#include "pitch_lqr.hpp"
#include "roll_lqr.hpp"
#include "safety_supervisor.hpp"
#include "torque_allocator.hpp"

/**
 * @brief 轮腿车底盘：全身俯仰共模 LQR + 轮子偏航差动
 *
 * @note U=[轮共模,髋共模,膝共模]，一个共模量就是左右每个执行器各自要出的力矩
 */
class ChassisNode : public rclcpp::Node
{
public:
    using State         = PitchLqrController::State;
    using Control       = PitchLqrController::Control;
    using RollState     = RollLqrController::State;
    using RollControl   = RollLqrController::Control;

    // 轮子下标。数组全按这个顺序对齐，别再散着写
    static constexpr std::size_t kLeft       = 0;
    static constexpr std::size_t kRight      = 1;
    static constexpr std::size_t kWheelCount = 2;

    ChassisNode() : rclcpp::Node("chassis")
    {
        // 获取身高增益路径
        const std::string gain_directory  = ament_index_cpp::get_package_share_directory("project") + "/params/leg_gain/";
        // 获取底盘所需参数
        const ChassisConfiguration config = ChassisConfiguration::declare_from(*this, gain_directory);
        model_   = config.model;
        control_ = config.control;
        remote_  = config.remote;
        topics_  = config.topics;
        leg_     = config.leg;
        safety_  = config.safety;
        enabled_ = config.enabled;
        height_  = config.height;
        joint_names_    = config.wheel_joint_names;
        command_topics_ = config.wheel_command_topics;

        // 力矩分配器
        allocator_     = std::make_unique<TorqueAllocator>(config.allocator_config());
        // 身高增益调度器，掌管控制律参数
        gain_schedule_ = LqrGainSchedule(config.gain_paths, control_.dt, model_.wheel_radius, config.robot_description);
        // 身高运动学，将腿轮高度转换成角度或者将角度反解成高度
        height_kinematics_ = std::make_unique<LegHeightKinematics>(config.height_kinematics_config());
        if (height_.target_height_m < height_kinematics_->minimum_height() || height_.target_height_m > height_kinematics_->maximum_height())
        {
            throw std::invalid_argument("target_height_m 超出软限位可达范围");
        }
        height_reference_ = height_kinematics_->from_height(height_kinematics_->nominal_height());

        gravity_compensation_ = LegGravityCompensation(model_, height_);

        safety_supervisor_.configure(safety_, control_.fall_angle_rad, leg_.limit_lower, leg_.limit_upper);

        for (std::size_t i = 0; i < kWheelCount; ++i)
        {
            command_pub_[i] = create_publisher<std_msgs::msg::Float64>(command_topics_[i], 10);
        }
        for (std::size_t i = 0; i < leg_.command_topics.size(); ++i)
        {
            leg_command_pub_[i] = create_publisher<std_msgs::msg::Float64>(leg_.command_topics[i], 10);
        }

        // 回调只缓存最新一帧，控制律统一在定时器那一拍算：跟话题频率解耦，也不会多算
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(topics_.imu_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(topics_.joint_states_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(*msg); });

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(remote_.cmd_vel_topic, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(*msg); });

        parameter_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) { return on_parameters(parameters); });

        timer_ = create_wall_timer(std::chrono::duration<double>(control_.dt), [this]() { update(); });

        RCLCPP_INFO(get_logger(), "腿高范围 [%.4f, %.4f] m，目标 %.4f m，变化率 %.4f m/s",
                    height_kinematics_->minimum_height(), height_kinematics_->maximum_height(), height_.target_height_m, height_.maximum_rate_mps);
        RCLCPP_INFO(get_logger(), "已加载 %zu 个高度工作点的 Pitch/Roll LQR", gain_schedule_.points().size());
    }

private:
    ModelParams   model_;                       // 模型参数：轮半径、质量、惯量、重力；里程换算和重力前馈都读它
    ControlParams control_;                     // 控制参数：节拍 dt、符号、共模力矩上限、变化率
    RemoteParams  remote_;                      // 遥控参数：速度/偏航上限、身高步长、指令超时
    TopicParams   topics_;                      // 反馈话题名：IMU / 关节状态
    LegParams     leg_;                         // 腿接线与限位：四个关节名、指令话题、joint_sign、软限位
    SafetyParams  safety_;                      // 安全参数：反馈超时、节拍卡顿阈值、恢复驻留时长
    HeightParams  height_;                      // 腿高参数：目标高度、变化率、换算几何与重力前馈质量
    ActuatorEnableParams enabled_;              // 五路消融开关（轮/髋/膝共模 + 髋/膝差模），运行期可改

    ChassisState state_;                        // 最近一帧反馈，加上控制用的那四个量
    TeleopTarget teleop_;                       // 遥控目标值
    StatusFlags  flags_;                        // 反馈/标定/倒地，都是标志位
    
    PitchLqrController  pitch_lqr_;             // 俯仰全身 LQR：8 状态，出轮/髋/膝三路共模力矩
    RollLqrController   roll_lqr_;              // 横滚 LQR：吃左右腿差模四状态，出髋/膝差模力矩
    LqrGainSchedule     gain_schedule_;         // 按腿高插值的工作点增益表：K/X0/U0/几何雅可比一起换
    SafetySupervisor    safety_supervisor_;     // 安全状态机：反馈超时/卡拍/倒地 → 准不准控制、要不要重置

    std::unique_ptr<TorqueAllocator>     allocator_;                // 力矩分配器：把六路期望力矩折成满足限幅/变化率/软限位的实际值
    LegGravityCompensation               gravity_compensation_;     // 腿重力前馈：按当前腿高算髋/膝顶住自重的力矩
    LegHeightKinematics::Reference       height_reference_ {};      // 当前指令的腿高工作点（高度 + 髋/膝角），由目标按变化率逼近
    std::unique_ptr<LegHeightKinematics> height_kinematics_;        // 腿高 ↔ 髋膝角换算 + 软限位内的可达范围


    // 接线：哪个关节、指令发到哪个话题。两组都按下标对齐
    std::array<std::string, kWheelCount> joint_names_{};
    std::array<std::string, kWheelCount> command_topics_{};

    // 发给两个电机的指令（力矩 N·m）。电机在自己的进程里订这两个话题
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kWheelCount> command_pub_{};
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> leg_command_pub_{};

    // 最后一条 cmd_vel 的时刻。用 steady_clock：仿真时间会被暂停/复位
    std::chrono::steady_clock::time_point last_command_time_{};
    std::chrono::steady_clock::time_point last_imu_time_{};
    std::chrono::steady_clock::time_point last_joint_state_time_{};
    std::chrono::steady_clock::time_point last_update_time_{};
    bool has_update_time_ {false};

    // ---- ROS 句柄 ----
    rclcpp::TimerBase::SharedPtr                                      timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        cmd_vel_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

    /**
     * @brief 运行期改参数的入口：只有下面这几个键认，别的键一律放行不管
     *
     * @param parameters 这次要改的参数列表
     * @return rcl_interfaces::msg::SetParametersResult successful=false 时 ROS 会拒绝这次修改
     *
     * @note 认这三类：
     *       ① 五个消融开关（enable_*）：直接改，下一拍生效，用来复现"少了某一路会怎样"
     *       ② target_height_m：先按软限位可达范围夹一遍，不合法就整条拒绝并给出区间
     *       ③ height_rate_mps：必须有限且为正
     * @note 改 target_height_m 走 set_parameter 而不是直接赋值：校验、日志、以及"键盘那条
     *       脉冲路径"都汇到这一个入口，免得两处规则慢慢跑偏
     * @note 目标高度是**平滑**推进的：这里只改目标值，实际关节目标按 height_rate_mps 一拍拍逼近
     */
    rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters)
        {
            const std::string& name = parameter.get_name();
            try
            {
                if      (name == "enable_wheel_common")     enabled_.wheel_common = parameter.as_bool();
                else if (name == "enable_hip_common")       enabled_.hip_common   = parameter.as_bool();
                else if (name == "enable_knee_common")      enabled_.knee_common  = parameter.as_bool();
                else if (name == "enable_hip_diff")         enabled_.hip_diff     = parameter.as_bool();
                else if (name == "enable_knee_diff")        enabled_.knee_diff    = parameter.as_bool();
                else if (name == "target_height_m")
                {
                    const double target = parameter.as_double();
                    if (!std::isfinite(target) || target < height_kinematics_->minimum_height() || target > height_kinematics_->maximum_height())
                    {
                        result.successful = false;
                        result.reason = "target_height_m 超出软限位可达范围 [" + std::to_string(height_kinematics_->minimum_height()) + ", " +
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
                else continue;   // 别的参数不归这里管，放行
                RCLCPP_WARN(get_logger(), "执行器消融开关变更：%s=%s", name.c_str(), parameter.as_bool() ? "true" : "false");
            }
            catch (const rclcpp::ParameterTypeException& error)
            {
                // 类型不对（比如给布尔参数塞了字符串）：整条拒绝，别改了一半
                result.successful = false;
                result.reason = error.what();
                return result;
            }
        }
        return result;
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
        if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.linear.z) || !std::isfinite(msg.angular.z))
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
            const double requested_height = std::clamp(height_.target_height_m + height_direction * remote_.height_step_m,
                                                        height_kinematics_->minimum_height(), 
                                                        height_kinematics_->maximum_height());

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

    /**
     * @brief 把跟时间相关的控制状态清零（故障恢复、重新接管时调）
     *
     * @note 清四样：遥控目标、位移原点标定标志、轮速、分配器的变化率历史。
     *       倒地或超时期间轮子在空转、状态已经不可信，不清就是拿着一堆假数据接着控
     * @note 腿高参考要重新锚一次：优先用**当前实测**的共模髋角反算高度，这样恢复后
     *       第一拍的关节目标不会从别处跳回来（差模、标定残差都留着，只重置基准）
     */
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

    /**
     * @brief 把安全状态机的判定落到这一拍上：打日志、按需重置状态、不准控制就发零
     *
     * @param decision 这一拍 SafetySupervisor 的判定
     * @return bool true = 允许继续控制，false = 已经发了零力矩，update() 该直接返回
     *
     * @note 事件只在"状态变了"那一拍给，所以这里的日志不会刷屏
     * @note 顺序：先按 reset_control_state 重置（可能发生在新接管那一拍），再判 allow_control ——
     *       重置完仍然不准发力的话就发零
     */
    bool apply_safety_decision(const SafetySupervisor::Decision& decision)
    {
        if (decision.event == SafetySupervisor::Event::kFaultEntered)
        {
            RCLCPP_WARN(get_logger(), "安全状态：%s，六路力矩立即清零",
                        SafetySupervisor::name(decision.state));
        }
        else if (decision.event == SafetySupervisor::Event::kFirstActivation)
        {
            RCLCPP_INFO(get_logger(), "首次反馈完整：标定位移原点并立即启用 LQR");
        }
        else if (decision.event == SafetySupervisor::Event::kRecoveryStarted)
        {
            RCLCPP_INFO(get_logger(), "反馈恢复，持续健康 %.3f s 后重新接管", safety_.recovery_dwell_s);
        }
        else if (decision.event == SafetySupervisor::Event::kRecovered)
        {
            RCLCPP_INFO(get_logger(), "安全状态恢复：重新标定位移原点并启用 LQR");
        }

        if (decision.reset_control_state)
        {
            reset_dynamic_state();
        }
        if (!decision.allow_control)
        {
            publish_zero_effort();
            return false;
        }
        return true;
    }

    /**
     * @brief 控制节拍：把最新一帧反馈喂给 LQR，把力矩发给两个轮子
     *        （回调只缓存，整形和求解都在这一拍里做，跟话题频率解耦）
     */
    void update()
    {
        const auto now = std::chrono::steady_clock::now();
        double control_period_s = 0.0;
        if (has_update_time_)
        {
            control_period_s = std::chrono::duration<double>(now - last_update_time_).count();
        }
        last_update_time_ = now;
        has_update_time_ = true;

        // 安全监督要三样：反馈来没来/新不新鲜、节拍被拖了没有、当前状态。
        // 反馈年龄用 steady_clock 算（仿真会暂停，仿真时间算不准这件事）
        SafetySupervisor::Input safety_input;
        safety_input.has_imu            = flags_.has_imu;
        safety_input.has_joint_state    = flags_.has_joint_state;
        safety_input.imu_age_s          = flags_.has_imu ? std::chrono::duration<double>(now - last_imu_time_).count() : 0.0;
        safety_input.joint_state_age_s  = flags_.has_joint_state ? std::chrono::duration<double>(now - last_joint_state_time_).count() : 0.0;
        safety_input.control_period_s   = control_period_s;
        safety_input.state = state_;

        if (!apply_safety_decision(safety_supervisor_.update(safety_input, now)))
        {
            return;
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

        // 腿高：先按变化率把"目标"变成这一拍实际允许的"指令高度"，再用它挑工作点。
        // 指令高度一帧最多动 maximum_rate_mps * dt，所以换腿高不会一步跳过去
        const LegHeightKinematics::Reference previous_height_reference = height_reference_;
        const double commanded_height = LegHeightKinematics::move_towards(previous_height_reference.height, height_.target_height_m, 
                                                                     height_.maximum_rate_mps,             control_.dt);

        height_reference_ = height_kinematics_->from_height(commanded_height);

        // 按腿高插值出这一拍用的 Pitch/Roll 参数（K/X0/U0/几何雅可比一起换）
        const auto scheduled_gain = gain_schedule_.at(height_reference_.height);
        pitch_lqr_.configure(scheduled_gain.pitch);
        roll_lqr_.configure(scheduled_gain.roll);

        // 对称伸缩下膝角是髋角的 -2 倍（q_k = -2 q_h），所以膝的目标角速度也差 -2 倍 ——
        // 不这么给的话 LQR 会把"膝没跟上"当成扰动去修，等于自己在跟自己较劲
        const double target_hip_rate = (height_reference_.hip - previous_height_reference.hip) / control_.dt;
        const double target_knee_rate = -2.0 * target_hip_rate;

        // 参考点 Xref：位移/速度用遥控目标叠在工作点 X0 上，四个腿量用这一拍的高度指令。
        // 注意不是"零"：X0 是该腿高下的工作点，遥控量是相对它的偏移
        State target = pitch_lqr_.parameters().x0;
        target(0) += teleop_.position;
        target(1) += teleop_.velocity;
        target(4) = height_reference_.hip;
        target(5) = target_hip_rate;
        target(6) = height_reference_.knee;
        target(7) = target_knee_rate;

        // 实测腿角拆成共模 + 差模：共模（左右平均）喂 Pitch，差模（左右之差的一半）喂 Roll。
        // 顺序固定 [左髋, 右髋, 左膝, 右膝]
        const double hip_position  = 0.5 * (state_.leg_position_rad[0]   + state_.leg_position_rad[1]);
        const double hip_velocity  = 0.5 * (state_.leg_velocity_radps[0] + state_.leg_velocity_radps[1]);
        const double knee_position = 0.5 * (state_.leg_position_rad[2]   + state_.leg_position_rad[3]);
        const double knee_velocity = 0.5 * (state_.leg_velocity_radps[2] + state_.leg_velocity_radps[3]);

        const double hip_difference       = 0.5 * (state_.leg_position_rad[0]   - state_.leg_position_rad[1]);
        const double hip_difference_rate  = 0.5 * (state_.leg_velocity_radps[0] - state_.leg_velocity_radps[1]);
        const double knee_difference      = 0.5 * (state_.leg_position_rad[2]   - state_.leg_position_rad[3]);
        const double knee_difference_rate = 0.5 * (state_.leg_velocity_radps[2] - state_.leg_velocity_radps[3]);

        State state;
        state << state_.wheel_position - state_.position_origin, state_.wheel_velocity,
                 state_.pitch, state_.pitch_rate, hip_position, hip_velocity, knee_position, knee_velocity;

        // U = U0 - K(X-Xref)。U 里的共模量是"每侧执行器各出这么多"，拆回左右时不再除 2。
        Control feedforward = pitch_lqr_.parameters().u0;
        const auto [height_hip_u0, height_knee_u0] = gravity_compensation_.effort(height_reference_.hip, height_reference_.knee);
        feedforward(1) = height_hip_u0;
        feedforward(2) = height_knee_u0;

        Control common = pitch_lqr_.update(state, target, feedforward, control_.control_sign);
        if (!enabled_.wheel_common) common(0) = 0.0;
        if (!enabled_.hip_common)   common(1) = 0.0;
        if (!enabled_.knee_common)  common(2) = 0.0;

        RollState roll_state;
        roll_state << hip_difference, hip_difference_rate, knee_difference, knee_difference_rate;

        RollControl leg_difference = roll_lqr_.update(roll_state, control_.control_sign);

        if (!enabled_.hip_diff) leg_difference(0) = 0.0;
        if (!enabled_.knee_diff) leg_difference(1) = 0.0;
        const double predicted_roll      = roll_lqr_.predict_roll(hip_difference, knee_difference);
        const double predicted_roll_rate = roll_lqr_.predict_roll_rate(hip_difference_rate, knee_difference_rate);

        // 转向：左右轮反向偏置，力矩差产生偏航力矩（正 = 左转）。反馈只有角速度、没有偏航角，
        // 所以这条环管的是"转多快"，不是"转到哪"
        const double differential = std::clamp(remote_.yaw_kp * (teleop_.yaw_rate - state_.yaw_rate), -remote_.max_differential, remote_.max_differential);

        TorqueAllocator::Input allocation_input;
        allocation_input.wheel_common = common(0);
        allocation_input.wheel_diff   = -differential;  // 保持原偏航接线：左=c-d、右=c+d
        allocation_input.hip_common   = common(1);
        allocation_input.hip_diff     = leg_difference(0);
        allocation_input.knee_common  = common(2);
        allocation_input.knee_diff    = leg_difference(1);

        TorqueAllocator::Feedback allocation_feedback;
        allocation_feedback.position  = state_.leg_position_rad;
        allocation_feedback.velocity  = state_.leg_velocity_radps;
        const TorqueAllocator::Result allocation = allocator_->allocate(allocation_input, allocation_feedback);
        if (allocation.diagnostics.invalid_input)
        {
            apply_safety_decision(safety_supervisor_.trip(SafetySupervisor::State::kInvalidState));
            return;
        }
        publish_allocated_effort(allocation.effort);

        if (allocation.diagnostics.priority_limited || allocation.diagnostics.effort_limited ||
            allocation.diagnostics.rate_limited     || allocation.diagnostics.soft_limit_active)
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

    /**
     * @brief 六路力矩全部发 0（故障、等待反馈、恢复观察期都走这里）
     *
     * @note 必须真的发出去而不是"不发"：电机节点那边的指令超时只有 50 ms，
     *       靠它兜底等于把停车时机交给了另一个进程的时钟
     */
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

    /**
     * @brief 把分配器算好的六路力矩发出去
     *
     * @param effort 六路力矩 N·m，顺序 [左轮, 右轮, 左髋, 右髋, 左膝, 右膝]
     *
     * @note 轮子那两路直接发；腿的四路要乘 joint_sign 翻到电机的正方向约定上
     *       （顺序上腿的四个正好接在轮子后面，所以是 effort[i + kWheelCount]）
     */
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
