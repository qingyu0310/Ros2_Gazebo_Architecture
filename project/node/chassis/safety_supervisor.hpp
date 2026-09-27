/**
 * @file safety_supervisor.hpp
 * @brief 底盘反馈、姿态、关节限位与故障恢复的统一安全状态机
 *
 * @note 它只回答两个问题：这一拍准不准发力（allow_control）、要不要把控制器状态清零重来
 *       （reset_control_state）。力矩怎么裁剪是 torque_allocator 的事，这里不发力矩
 * @note 故障分门别类（超时 / 非有限 / 倒地 / 越硬限位 / 节拍异常），日志里能直接看出是哪一类 ——
 *       全都归成一个"故障"的话，现场只能靠猜
 * @note 关节限位这一层查的是**硬限位**：软限位和恢复力矩是分配器那边的事，这里只负责
 *       "已经撞到硬限位了"这种必须立刻停的情况
 */
#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "chassis_types.hpp"

/**
 * @brief 安全状态机：吃反馈的健康状况和底盘状态，吐"这一拍准不准控制、要不要重置状态"
 */
class SafetySupervisor
{
public:
    using Clock = std::chrono::steady_clock;   // 用真实时钟：仿真时间会暂停/复位，拿它判超时会判错

    /**
     * @brief 状态机的状态：kActive 是唯一允许控制的，其余都是某一类故障（或还没起步）
     */
    enum class State
    {
        kWaitingFeedback,    // 还没收到过 IMU/关节反馈，不发力也不算故障
        kRecovering,         // 故障消失后的观察期：连续健康够久才回 kActive
        kActive,             // 正常控制
        kFeedbackTimeout,    // IMU 或关节反馈超时
        kInvalidState,       // 状态里有 NaN/Inf
        kFallen,             // |pitch| 或 |roll| 超限，认定倒地
        kJointLimit,         // 某个腿关节越过硬限位
        kTimingFault         // 控制周期超上限，节拍被拖了
    };

    /**
     * @brief 状态切换要告诉调用方的事：只在"变了"的那一拍给事件，便于打一条日志
     */
    enum class Event
    {
        kNone,
        kFaultEntered,       // 进入故障（含换了一类故障）
        kFirstActivation,    // 第一次从"等反馈"进 kActive
        kRecoveryStarted,    // 进入观察期
        kRecovered           // 观察期满，重新接管
    };

    /**
     * @brief 状态机这一拍的输入
     */
    struct Input
    {
        bool has_imu             {false};   // 收到过 IMU 没有
        bool has_joint_state     {false};   // 收到过"两个轮子都在"的一帧没有
        double imu_age_s         {0.0};     // 最后一帧 IMU 到现在多久 s（没收到过就是无穷大）
        double joint_state_age_s {0.0};     // 同理，关节状态
        double control_period_s  {0.0};     // 上一拍到现在多久 s，用来查节拍卡顿
        ChassisState state       {};        // 最近一帧反馈解出来的状态
    };

    /**
     * @brief 状态机的判定结果
     */
    struct Decision
    {
        State state {State::kWaitingFeedback};  // 这一拍处在哪个状态
        Event event {Event::kNone};             // 状态有没有变
        bool allow_control {false};             // 准不准发力（力矩该不该发出去）
        bool reset_control_state {false};       // 要不要把里程原点、目标、分配器历史清零重来
    };

    /**
     * @brief 建一个未配置的实例
     *
     * @note 未配置时调 update()/trip() 会抛；正常路径是用下面那个构造器
     */
    SafetySupervisor() = default;

    /**
     * @brief 用安全参数、倒地角和四个腿关节的硬限位建实例
     *
     * @param safety 反馈超时、节拍上限、恢复驻留
     * @param fall_angle_rad |pitch| 或 |roll| 超过它就判倒地 rad
     * @param joint_lower 四个腿关节的硬限位下限 rad，顺序 [左髋, 右髋, 左膝, 右膝]
     * @param joint_upper 硬限位上限 rad
     */
    SafetySupervisor(const SafetyParams& safety, double fall_angle_rad, const std::vector<double>& joint_lower, const std::vector<double>& joint_upper)
    {
        configure(safety, fall_angle_rad, joint_lower, joint_upper);
    }

    /**
     * @brief 配置（或重新配置）状态机
     *
     * @param safety 安全参数
     * @param fall_angle_rad 倒地角阈值 rad，必须为正
     * @param joint_lower 四个硬限位下限，长度必须为 4
     * @param joint_upper 四个硬限位上限，长度必须为 4
     *
     * @throw std::invalid_argument 任一时间/角度非有限或非正、数组长度不是 4、上下限反了时抛
     */
    void configure(const SafetyParams& safety, double fall_angle_rad, const std::vector<double>& joint_lower, const std::vector<double>& joint_upper)
    {
        if (!std::isfinite(safety.feedback_timeout_s) || safety.feedback_timeout_s <= 0.0 || !std::isfinite(safety.max_control_period_s) || 
            safety.max_control_period_s <= 0.0 || !std::isfinite(safety.recovery_dwell_s) || safety.recovery_dwell_s < 0.0 || 
            !std::isfinite(fall_angle_rad)  || fall_angle_rad <= 0.0 || joint_lower.size() != 4 || joint_upper.size() != 4)
        {
            throw std::invalid_argument("安全监督配置无效");
        }
        safety_ = safety;
        fall_angle_rad_ = fall_angle_rad;

        for (std::size_t i = 0; i < 4; ++i)
        {
            if (!std::isfinite(joint_lower[i]) || !std::isfinite(joint_upper[i]) ||
                joint_lower[i] >= joint_upper[i])
            {
                throw std::invalid_argument("安全监督关节限位无效");
            }
            joint_lower_[i] = joint_lower[i];
            joint_upper_[i] = joint_upper[i];
        }
        configured_ = true;
    }

    /**
     * @brief 跑一拍状态机
     *
     * @param input 这一拍的健康状况 + 状态
     * @param now 这一拍的时刻（steady_clock，用来算恢复观察期）
     * @return Decision 准不准发力、要不要重置控制状态、状态有没有变
     *
     * @throw std::logic_error 还没 configure
     *
     * @note 分支顺序就是优先级：先判健康（不健康立刻置故障并停手），健康了才看是不是
     *       "一直正常" / "第一次接管" / "刚进观察期" / "观察期满"
     * @note 故障那一拍一律带回 reset_control_state = true：倒地/超时期间轮子在空转、
     *       里程和目标都已经不可信，重新接管时必须重标原点
     */
    Decision update(const Input& input, const Clock::time_point& now)
    {
        require_configured();

        // 健康检查不过：立刻置成对应的故障状态，停手，并要求调用方把控制状态清零
        const State health = evaluate(input);
        if (health != State::kActive)
        {
            const bool changed = state_ != health;
            state_ = health;
            recovery_started_ = false;
            return {state_, changed ? Event::kFaultEntered : Event::kNone, false, true};
        }

        // 一直正常：什么都不用做
        if (state_ == State::kActive)
        {
            return {state_, Event::kNone, true, false};
        }
        // 开机以来第一次健康：直接接管（首个目标由调用方重标，所以带 reset）
        if (!ever_active_)
        {
            ever_active_ = true;
            state_ = State::kActive;
            return {state_, Event::kFirstActivation, true, true};
        }
        // 刚从故障里出来：先记下健康起点，进观察期，这期间仍不发力
        if (!recovery_started_)
        {
            healthy_since_ = now;
            recovery_started_ = true;
            state_ = State::kRecovering;
            return {state_, Event::kRecoveryStarted, false, false};
        }
        // 观察期还没满：继续等（抖动一下就要重新计时，因为上面每拍都会重置 healthy_since_）
        if (std::chrono::duration<double>(now - healthy_since_).count() < safety_.recovery_dwell_s)
        {
            return {state_, Event::kNone, false, false};
        }

        // 观察期满：重新接管，同样要求重置控制状态
        recovery_started_ = false;
        state_ = State::kActive;
        return {state_, Event::kRecovered, true, true};
    }

    /**
     * @brief 从外部立刻触发一个安全故障
     *
     * @param reason 故障类型，必须是故障态（kActive / kRecovering 不合法）
     * @return Decision 跟 update() 一样：停手 + 要求重置控制状态
     *
     * @throw std::logic_error 还没 configure
     * @throw std::invalid_argument reason 是 kActive 或 kRecovering 时抛
     *
     * @note 给控制计算和力矩分配用：那两处发现算不动、越限、非有限时，别自己悄悄降级，
     *       直接把状态机摁进故障，后面统一走恢复流程
     */
    Decision trip(State reason)
    {
        require_configured();
        if (reason == State::kActive || reason == State::kRecovering)
        {
            throw std::invalid_argument("不能把正常或恢复状态作为故障触发");
        }
        const bool changed = state_ != reason;
        state_ = reason;
        recovery_started_ = false;
        return {state_, changed ? Event::kFaultEntered : Event::kNone, false, true};
    }

    /**
     * @brief 当前状态
     *
     * @return State 上一次 update()/trip() 之后的状态
     */
    State state() const
    {
        return state_;
    }

    /**
     * @brief 状态的中文名，打日志用
     *
     * @param state 状态
     * @return const char* 中文名；枚举加了新值却没补这里时返回"未知"
     */
    static const char* name(State state)
    {
        switch (state)
        {
            case State::kWaitingFeedback: return "等待反馈";
            case State::kRecovering:      return "恢复确认";
            case State::kActive:          return "正常控制";
            case State::kFeedbackTimeout: return "反馈超时";
            case State::kInvalidState:    return "状态非有限";
            case State::kFallen:          return "姿态倒地";
            case State::kJointLimit:      return "关节越过硬限位";
            case State::kTimingFault:     return "控制周期异常";
        }
        return "未知";
    }

private:
    bool configured_ {false};                                     // 没配置过就抛
    bool ever_active_ {false};                                    // 是否已经接管过一次（区分首激活和恢复）
    bool recovery_started_ {false};                               // 是否在观察期里

    double fall_angle_rad_ {0.0};                                 // 倒地角阈值 rad
    State state_ {State::kWaitingFeedback};                       // 当前状态，从"等反馈"起步

    SafetyParams safety_ {};                                      // 超时/节拍/恢复驻留
    std::array<double, 4> joint_lower_ {};                        // 四个腿关节硬限位下限，顺序 [左髋, 右髋, 左膝, 右膝]
    std::array<double, 4> joint_upper_ {};                        // 硬限位上限

    Clock::time_point healthy_since_ {};                          // 观察期的健康起点

    /**
     * @brief 健康检查：按优先级逐项判，返回第一个不满足的状态
     *
     * @param input 这一拍的输入
     * @return State kActive（全部通过）或具体故障
     *
     * @note 顺序有讲究：先看"收到过没有"（开机头几拍不该算故障），再看反馈新不新鲜、
     *       节拍有没有被拖，然后才敢用状态里的数（先查有限性，避免拿 NaN 去比大小）
     */
    State evaluate(const Input& input) const
    {
        if (!input.has_imu || !input.has_joint_state)
        {
            return State::kWaitingFeedback;
        }
        if (input.imu_age_s > safety_.feedback_timeout_s || input.joint_state_age_s > safety_.feedback_timeout_s)
        {
            return State::kFeedbackTimeout;
        }
        if (input.control_period_s > safety_.max_control_period_s)
        {
            return State::kTimingFault;
        }
        if (!finite(input.state))
        {
            return State::kInvalidState;
        }
        // 俯仰和横滚任一超限都算倒地：倒着躺、侧着躺都得停手
        if (std::abs(input.state.pitch) > fall_angle_rad_ || std::abs(input.state.roll) > fall_angle_rad_)
        {
            return State::kFallen;
        }
        // 硬限位：撞上了说明控制器已经压不住，立刻停手（软限位和恢复力矩在分配器那边）
        for (std::size_t i = 0; i < input.state.leg_position_rad.size(); ++i)
        {
            if (input.state.leg_position_rad[i] < joint_lower_[i] ||
                input.state.leg_position_rad[i] > joint_upper_[i])
            {
                return State::kJointLimit;
            }
        }
        return State::kActive;
    }

    /**
     * @brief 状态里的数是不是都是有限数（角度、角速度、轮子位移速度、四个腿关节）
     *
     * @param state 待查状态
     * @return bool 全有限返回 true
     */
    static bool finite(const ChassisState& state)
    {
        if (!std::isfinite(state.roll)       || !std::isfinite(state.roll_rate) || !std::isfinite(state.pitch)          || 
            !std::isfinite(state.pitch_rate) || !std::isfinite(state.yaw_rate)  || !std::isfinite(state.wheel_position) ||
            !std::isfinite(state.wheel_velocity))
        {
            return false;
        }
        for (std::size_t i = 0; i < state.leg_position_rad.size(); ++i)
        {
            if (!std::isfinite(state.leg_position_rad[i]) ||
                !std::isfinite(state.leg_velocity_radps[i]))
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 入口检查：没配置过就抛
     *
     * @throw std::logic_error 还没 configure
     *
     * @note 没配置时所有阈值都是 0/空，跑下去会把每一拍都判成故障或者全部放行，
     *       两种都比启动时报错难查
     */
    void require_configured() const
    {
        if (!configured_)
        {
            throw std::logic_error("SafetySupervisor 尚未配置");
        }
    }
};
