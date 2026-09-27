/**
 * @file roll_lqr.hpp
 * @author qingyu
 * @brief 轮腿机器人横滚差模 LQR：保存工作点参数并计算髋、膝差模力矩
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 * @note 只管"解算"这一段：参数从离线增益文件来（LqrGainSchedule 按腿高插值后 configure 进来），
 *       状态从 /joint_states 的差模来，力矩发回六路分配器 —— 中间不碰 ROS、不碰话题
 * @note 控制律是 U = U0 - control_sign * K * (X - X0)：跟 Pitch 那套一样带工作点偏移，
 *       不是裸的 U = -KX（裸写会让 LQR 长期输出一个固定偏差去顶重力，工作点就偏了）
 * @note 跟 Pitch 子系统的分工：Roll 用髋膝差模，轮差动留给偏航外环，不从这条路走
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <Eigen/Dense>

/**
 * @brief Roll 平面 4 状态、2 输入 LQR 控制器
 *
 * 状态顺序固定为：
 * [q_h_diff, q_h_diff_rate, q_k_diff, q_k_diff_rate]
 *
 * 输入顺序固定为：
 * [tau_h_diff, tau_k_diff]
 *
 * 两轮同时贴地时，车身 roll 不是独立状态，而由左右腿高差决定。因此控制状态只有四个
 * 关节差模量；IMU roll/roll_rate 留给 ChassisNode 做倒地保护和闭链几何诊断。
 */
class RollLqrController
{
public:
    static constexpr std::size_t kStateDim = 4;   // [q_h−, q̇_h−, q_k−, q̇_k−]
    static constexpr std::size_t kInputDim = 2;   // [τ_h−, τ_k−]

    using State         = Eigen::Matrix<double, kStateDim, 1>;        // 状态向量 X
    using Control       = Eigen::Matrix<double, kInputDim, 1>;        // 输入向量 U（差模力矩）
    using StateMatrix   = Eigen::Matrix<double, kStateDim, kStateDim>;
    using InputMatrix   = Eigen::Matrix<double, kStateDim, kInputDim>;
    using GainMatrix    = Eigen::Matrix<double, kInputDim, kStateDim>; // 反馈增益 K（2×4）
    using GeometryRow   = Eigen::Matrix<double, 1, 2>;                 // roll 对两个差模角的灵敏度

    /**
     * @brief 某个腿高工作点对应的在线控制参数
     *
     * @note 一次 configure 装一套；调度由 LqrGainSchedule 负责插值，这个类不自己换参数
     */
    struct Parameters
    {
        GainMatrix  gain {GainMatrix::Zero()};          // 离散 LQR 增益 K_roll
        State       x0   {State::Zero()};               // 差模工作点，正常为 0
        Control     u0   {Control::Zero()};             // 差模前馈，正常为 0
        GeometryRow roll_per_q {GeometryRow::Zero()};   // d(roll)/d[q_h_diff,q_k_diff]

        /**
         * @brief 四项参数是不是都是有限数
         *
         * @return bool 有一项含 NaN/Inf 就返回 false
         *
         * @note configure / interpolate 都用它当入口检查：插值出来带 NaN 说明端点上就不干净
         */
        bool all_finite() const
        {
            return gain.allFinite() && x0.allFinite() && u0.allFinite() && roll_per_q.allFinite();
        }
    };

    /**
     * @brief 装入当前腿高对应的 Roll 参数
     *
     * @param parameters 一套完整参数（K / X0 / U0 / roll_per_q）
     *
     * @throw std::invalid_argument 参数含 NaN 或 Inf
     */
    void configure(const Parameters& parameters)
    {
        if (!parameters.all_finite())
        {
            throw std::invalid_argument("Roll LQR 参数含 NaN 或 Inf");
        }
        parameters_ = parameters;
        configured_ = true;
    }

    /**
     * @brief 算这一拍的差模力矩
     *
     * @param state 当前差模状态（顺序 [q_h−, q̇_h−, q_k−, q̇_k−]）
     * @param control_sign 整体方向符号，装反了传 -1.0
     * @return Control 髋、膝差模力矩 [τ_h−, τ_k−]，单位 N·m（每侧各自 ±）
     *
     * @throw std::logic_error 还没 configure
     * @throw std::invalid_argument 状态或符号含 NaN/Inf
     */
    Control update(const State& state, double control_sign = 1.0) const
    {
        require_configured();
        if (!state.allFinite() || !std::isfinite(control_sign))
        {
            throw std::invalid_argument("Roll LQR 输入含 NaN 或 Inf");
        }
        return parameters_.u0 - control_sign * parameters_.gain * (state - parameters_.x0);
    }

    /**
     * @brief 用腿角差模预测闭链几何要求的车身横滚角
     *
     * @param hip_difference 髋差模角 rad，(q_hL - q_hR)/2
     * @param knee_difference 膝差模角 rad
     * @return double 预测的 roll rad
     *
     * @throw std::logic_error 还没 configure
     * @throw std::invalid_argument 输入含 NaN/Inf
     *
     * @note 用途是诊断：跟 IMU 实测的 roll 比，差得多说明腿角反馈或闭链关系不对，
     *       而不是控制参数不对
     */
    double predict_roll(double hip_difference, double knee_difference) const
    {
        require_configured();
        if (!std::isfinite(hip_difference) || !std::isfinite(knee_difference))
        {
            throw std::invalid_argument("Roll 几何预测输入含 NaN 或 Inf");
        }
        return (parameters_.roll_per_q * Eigen::Vector2d(hip_difference, knee_difference))(0);
    }

    /**
     * @brief 用腿角速度差模预测车身横滚角速度
     *
     * @param hip_difference_rate 髋差模角速度 rad/s
     * @param knee_difference_rate 膝差模角速度 rad/s
     * @return double 预测的 roll 角速度 rad/s
     *
     * @throw 跟 predict_roll 一样
     *
     * @note 直接用同一个 roll_per_q：roll 是腿角的（完整）函数，位置级和速度级的雅可比是同一个
     */
    double predict_roll_rate(double hip_difference_rate, double knee_difference_rate) const
    {
        return predict_roll(hip_difference_rate, knee_difference_rate);
    }

    /**
     * @brief 取当前装着的参数
     *
     * @return const Parameters& 参数引用（诊断、日志用）
     *
     * @throw std::logic_error 还没 configure
     */
    const Parameters& parameters() const
    {
        require_configured();
        return parameters_;
    }

    /**
     * @brief 对相邻两个腿高工作点的参数做线性插值
     *
     * @param lower 低腿高工作点的参数
     * @param upper 高腿高工作点的参数
     * @param ratio 插值比例 [0,1]：0 取 lower，1 取 upper
     * @return Parameters 插出来的参数
     *
     * @throw std::invalid_argument 两端参数含 NaN/Inf，或 ratio 不在 [0,1]
     *
     * @note 四组量都按同一个比例插：K/X0/U0/roll_per_q。只插 K 不插 roll_per_q 的话，
     *       几何诊断会按旧腿高的关系去比，反而给出误导性的"不一致"
     */
    static Parameters interpolate(const Parameters& lower, const Parameters& upper, double ratio)
    {
        if (!lower.all_finite() || !upper.all_finite() || !std::isfinite(ratio) || ratio < 0.0 || ratio > 1.0)
        {
            throw std::invalid_argument("Roll LQR 插值参数无效");
        }

        const double inverse = 1.0 - ratio;
        Parameters result;
        result.gain = inverse * lower.gain + ratio * upper.gain;
        result.x0   = inverse * lower.x0 + ratio * upper.x0;
        result.u0   = inverse * lower.u0 + ratio * upper.u0;
        result.roll_per_q = inverse * lower.roll_per_q + ratio * upper.roll_per_q;
        return result;
    }

private:
    Parameters parameters_ {};   // 上一次 configure 装进来的那套
    bool configured_ {false};    // 没配置就调 update()/predict_* 会抛，别拿零增益当"不动"

    /**
     * @brief 入口检查：没配置过就抛
     *
     * @throw std::logic_error 还没装参数
     *
     * @note 宁可启动就崩，也别让零增益的 K 悄悄把差模力矩全发成 0（那等于 Roll 环不存在）
     */
    void require_configured() const
    {
        if (!configured_)
        {
            throw std::logic_error("Roll LQR 尚未配置工作点参数");
        }
    }
};
