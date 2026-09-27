/**
 * @file motor.hpp
 * @author qingyu
 * @brief 关节电机模块：把上层给的目标（力矩或速度）整形成能给执行器的一帧力矩，管住限幅和指令死区
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace modules::motor {

/**
 * @brief 控制模式：这一路电机认哪种目标值
 *
 * @note 对应 gazebo 层两个互斥的插件宏：kEffort 配 torque_motor（ApplyJointForce，闭环在
 *       你的节点里），kSpeed 配 velocity_motor 的 use_force_commands=true 那种用法。本项目
 *       走的是 LQR + torque_motor 的力矩路线，kSpeed 是留着对比/调试的那条腿
 */
enum class Mode : std::uint8_t
{
    kEffort = 0,  // 目标值 = 关节力矩，N·m，夹到限幅后直通
    kSpeed  = 1,  // 目标值 = 关节角速度，rad/s，本模块内跑 PI 换成力矩
};

/**
 * @brief 一拍输入（输入契约）：上层每拍填一次，交给 Motor::update()
 *
 * @note 这一层不碰时钟也不碰中间件：时间戳由上层填纳秒，"指令年龄"由上层用自己的时钟算
 *       —— 回调到定时器之间隔了多久只有上层知道 —— 本模块只按填进来的数干活
 */
struct MotorTick
{
    std::int64_t stamp_ns{0};                                               // 本拍时刻，纳秒；速度环算 dt 用
    double       command_age_s{std::numeric_limits<double>::infinity()};    // 最新力矩指令到现在的时长，秒；判死区用。默认无穷大 = 一条指令都还没收到
    double       target_effort_nm{0.0};                                     // 目标关节力矩，N·m（kEffort 模式用）
    double       target_speed_radps{0.0};                                   // 目标关节角速度，rad/s（kSpeed 模式用）
    double       speed_radps{0.0};                                          // 关节反馈角速度，rad/s
    bool         has_speed_feedback{false};                                 // 上面那个反馈是不是有效值
};

/**
 * @brief 一拍输出（输出契约）：node 直接拿 effort_nm 填 cmd_force 那一帧
 */
struct MotorOutput
{
    std::int64_t stamp_ns{0};                                               // 本拍时刻，原样透传
    double       effort_nm{0.0};                                            // 要施加到关节上的力矩，N·m
    bool         active{false};                                             // 本拍是否真在按指令出力：false 说明这个 0 是"安全侧默认"，不是指令值
};

/**
 * @brief 单关节电机：限幅、指令死区、速度环都在这里
 *
 * @note 一个实例管一个关节：左轮右轮是两个实例，别把两路塞进一个对象里
 * @note 不是电机模型：没有力矩常数 Kt、反电动势、电流环、力矩-转速曲线，除了夹紧不做任何
 *       非线性 —— 跟 gazebo 层 torque_motor 宏保持一致（仿真里唯一的硬上限是 URDF 的
 *       <limit effort>，所以这里的 effort_limit 要跟它对齐，或者更小）
 * @note 本文件不含 ROS：话题名、消息类型、哪来的回调，都由上层 node 决定
 */
class Motor
{
public:
    Motor() = default;

    /**
     * @brief 选控制模式
     *
     * @param mode 力矩直通还是速度闭环
     */
    void set_mode(Mode mode) { mode_ = mode; }

    /**
     * @brief 设置力矩上限（取绝对值）
     *
     * @param effort_nm 上限，N·m
     */
    void set_effort_limit(double effort_nm) { effort_limit_nm_ = std::abs(effort_nm); }

    /**
     * @brief 设置速度目标上限（取绝对值）：速度模式下目标值先夹到这个范围
     *
     * @param speed_radps 上限，rad/s
     */
    void set_speed_limit(double speed_radps) { speed_limit_radps_ = std::abs(speed_radps); }

    /**
     * @brief 设置速度环 PI 增益
     *
     * @param kp 比例增益，N·m/(rad/s)
     * @param ki 积分增益，N·m/rad
     */
    void set_speed_gains(double kp, double ki)
    {
        speed_kp_ = kp;
        speed_ki_ = ki;
    }

    /**
     * @brief 设置指令死区时长：超过这么久没收到新的力矩指令，输出就归零
     *
     * @param timeout_s 死区时长，秒；只在 kEffort 模式下判
     *
     * @note 速度路线不判死区（跟 lqr 那个节点一致）：kSpeed 下目标速度一直有效，要停就发 0
     */
    void set_command_timeout(double timeout_s) { command_timeout_s_ = std::max(0.0, timeout_s); }

    /**
     * @brief 清空积分与时间戳，回到未初始化状态
     */
    void reset()
    {
        output_         = MotorOutput{};
        integral_       = 0.0;
        last_stamp_ns_  = 0;
        has_last_stamp_ = false;
    }

    /**
     * @brief 推进一拍：按模式算出这一拍要发的力矩
     *
     * @param tick 本拍输入
     *
     * @note 没有返回值：电机这一路每拍都必须有输出。不该出力的时候（死区超时、速度模式还没
     *       拿到关节反馈）输出的是 0，由 active 标出来 —— 这跟 IMU 模块那种"这帧不要"的
     *       语义不一样，宁可发 0 也不能不发
     */
    void update(const MotorTick& tick)
    {
        const double dt_s = tick_dt_s(tick.stamp_ns);

        output_.stamp_ns = tick.stamp_ns;

        if (mode_ == Mode::kEffort)
        {
            update_effort(tick);
            return;
        }

        update_speed(tick, dt_s);
    }

    /**
     * @brief 本拍输出
     *
     * @return const MotorOutput& 本拍输出
     */
    const MotorOutput& output() const { return output_; }

    /**
     * @brief 本拍要发的力矩，省得每次写 output().effort_nm
     *
     * @return double 力矩，N·m
     */
    double effort_nm() const { return output_.effort_nm; }

private:
    /**
     * @brief 本拍与上一拍的时间差
     *
     * @param stamp_ns 本拍时刻，纳秒
     * @return double 时间差，秒；第一拍（没有上一拍）和时刻没前进的都返回 0
     */
    double tick_dt_s(std::int64_t stamp_ns)
    {
        if (!has_last_stamp_)
        {
            last_stamp_ns_  = stamp_ns;
            has_last_stamp_ = true;
            return 0.0;
        }

        if (stamp_ns <= last_stamp_ns_)
        {
            return 0.0;
        }

        const double dt_s = static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9;
        last_stamp_ns_    = stamp_ns;
        return dt_s;
    }

    /**
     * @brief 力矩模式：只判死区和限幅
     *
     * @param tick 本拍输入
     */
    void update_effort(const MotorTick& tick)
    {
        // NaN/Inf 的指令按"没在出力"算：输出是 0，但这个 0 是安全侧默认，active 得跟着是 false
        const bool fresh = std::isfinite(tick.target_effort_nm) && tick.command_age_s <= command_timeout_s_;

        output_.effort_nm = fresh ? clamp_effort(tick.target_effort_nm) : 0.0;
        output_.active    = fresh;
    }

    /**
     * @brief 速度模式：PI 把速度误差换成力矩
     *
     * @param tick  本拍输入
     * @param dt_s  本拍时间差，秒
     *
     * @note 没有关节反馈就输出 0 且不动积分：拿一个"速度为 0"的假反馈去跑 PI，
     *       积分会白白顶到限幅，等真反馈回来时反向冲一下
     */
    void update_speed(const MotorTick& tick, double dt_s)
    {
        if (!tick.has_speed_feedback)
        {
            output_.effort_nm = 0.0;
            output_.active    = false;
            return;
        }

        const double target = clamp_speed(tick.target_speed_radps);
        const double error  = target - tick.speed_radps;

        // 积分抗饱和：积分项夹到"光靠积分就能顶到力矩上限"的范围，超了就是白攒
        const double integral_limit = effort_limit_nm_ / std::max(std::abs(speed_ki_), 1e-9);
        integral_ = std::clamp(integral_ + error * dt_s, -integral_limit, integral_limit);

        output_.effort_nm = clamp_effort(speed_kp_ * error + speed_ki_ * integral_);
        output_.active    = true;
    }

    /**
     * @brief 力矩夹紧，非有限值（NaN/Inf）当 0 处理
     *
     * @param effort_nm 原始力矩，N·m
     * @return double 夹紧后的力矩，N·m
     */
    double clamp_effort(double effort_nm) const
    {
        if (!std::isfinite(effort_nm))
        {
            return 0.0;
        }
        return std::clamp(effort_nm, -effort_limit_nm_, effort_limit_nm_);
    }

    /**
     * @brief 速度目标夹紧，非有限值（NaN/Inf）当 0 处理
     *
     * @param speed_radps 原始速度，rad/s
     * @return double 夹紧后的速度，rad/s
     */
    double clamp_speed(double speed_radps) const
    {
        if (!std::isfinite(speed_radps))
        {
            return 0.0;
        }
        return std::clamp(speed_radps, -speed_limit_radps_, speed_limit_radps_);
    }

    Mode   mode_{Mode::kEffort};          // 默认走力矩路线，跟本项目一致
    // 跟 URDF 的 <limit effort> 对齐（wheel.xacro 的 effort）。
    // 给得比"够用"大一个量级：40mm 轮子、整车约 1kg 时轮胎附着极限才 0.1 N·m，
    // 5.0 是它的 50 倍 —— 电机永远不该是瓶颈，受限的只能是轮胎打滑。
    // 这个上限越小，能落地的力矩越小，表现就是"车没劲、转不动"
    double effort_limit_nm_{5.0};
    // 关节转速硬上限，跟 wheel.xacro 的 velocity 对齐。60 rad/s = 2.4 m/s（r=0.04），
    // 而遥控给的最大直线速度只要 25 rad/s，留了一倍多余量 —— 别让它成为瓶颈
    double speed_limit_radps_{60.0};

    // 速度环增益。定这两个数的依据是"电机要能盖住轮胎的附着极限"：
    // 40mm 轮子、单轮法向力 2.45 N（整车约 1kg）时附着极限力矩约 0.1 N·m。
    // 转弯时轮速目标只有 ±1.5 rad/s，Kp = 0.10 就是 0.15 N·m，一按下去立刻顶满附着，
    // 不用等积分慢慢攒 —— Kp 给 0.02 时比例项只有 0.03 N·m，正好卡在启动阈值上，这就是"慢半拍"。
    // 上限约束：Kp·dt/J 是每拍消掉的误差比例（J 是轮子自身转动惯量 0.00027），
    // dt = 1ms 时 Kp = 0.10 是 37%/拍；轮子贴地时有整车惯量当负载，余量很大，
    // 但空转（腾空、脱开地面）时余量不大，再往大调得先在仿真里看有没有嗡嗡抖
    double speed_kp_{0.10};
    double speed_ki_{1.0};                // 积分时间 Kp/Ki = 0.1 s；抗饱和上限 = effort_limit/Ki，跟着一起放大
    double command_timeout_s_{0.05};      // 50 ms 没新指令就松力矩

    double integral_{0.0};                // 速度环积分项

    std::int64_t last_stamp_ns_{0};       // 上一拍时间戳
    bool         has_last_stamp_{false};  // 是否已经有上一拍

    MotorOutput  output_{};               // 最新一拍输出
};

} // namespace modules::motor
