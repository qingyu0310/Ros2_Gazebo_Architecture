/**
 * @file pitch_lqr.hpp
 * @author qingyu
 * @brief 轮腿机器人纵向共模 LQR：保存工作点参数并计算轮、髋、膝共模力矩
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <Eigen/Dense>

/**
 * @brief Pitch 平面 8 状态、3 输入 LQR 控制器
 *
 * 状态顺序固定为：
 * [s, s_dot, pitch, pitch_rate,
 *  q_h_common, q_h_common_rate, q_k_common, q_k_common_rate]
 *
 * 输入顺序固定为：
 * [tau_w_common, tau_h_common, tau_k_common]
 *
 * 本类只负责纯控制律，不读取 YAML、不订阅 ROS 话题，也不做执行器限幅。
 * ChassisNode 负责组装状态和目标，TorqueAllocator 负责把共模/差模恢复成六路力矩。
 */
class PitchLqrController
{
public:
    static constexpr std::size_t kStateDim = 8;
    static constexpr std::size_t kInputDim = 3;

    using State      = Eigen::Matrix<double, kStateDim, 1>;
    using Control    = Eigen::Matrix<double, kInputDim, 1>;
    using GainMatrix = Eigen::Matrix<double, kInputDim, kStateDim>;

    /**
     * @brief 某个腿高工作点对应的在线控制参数
     *
     * @note 一次 configure 装一套；换腿高由 LqrGainSchedule 插值后重新装，这个类不自己换参数
     */
    struct Parameters
    {
        GainMatrix gain {GainMatrix::Zero()};       // 离散 LQR 增益 K
        State      x0   {State::Zero()};            // 线性化工作点 X0
        Control    u0   {Control::Zero()};          // 静态平衡前馈 U0

        /**
         * @brief 三组参数是不是都是有限数
         *
         * @return bool 有一项含 NaN/Inf 就返回 false
         *
         * @note configure / interpolate 都拿它当入口检查：插值出来带 NaN，说明端点上就不干净
         */
        bool all_finite() const
        {
            return gain.allFinite() && x0.allFinite() && u0.allFinite();
        }
    };

    /**
     * @brief 装入当前腿高对应的参数
     * @throws std::invalid_argument 参数含 NaN 或 Inf
     */
    void configure(const Parameters& parameters)
    {
        if (!parameters.all_finite())
        {
            throw std::invalid_argument("Pitch LQR 参数含 NaN 或 Inf");
        }
        parameters_ = parameters;
        configured_ = true;
    }

    /**
     * @brief 计算共模力矩 U = Uff - control_sign * K * (X - Xref)
     *
     * @param state 当前 8 状态 X
     * @param target 当前参考状态 Xref；允许在 X0 基础上修改位置、速度和腿高目标
     * @param feedforward 当前前馈 Uff；允许用实时几何重算髋膝重力前馈
     * @param control_sign 执行器总符号，正常为 +1
     */
    Control update(const State& state, const State& target, const Control& feedforward, double control_sign = 1.0) const
    {
        require_configured();
        if (!state.allFinite() || !target.allFinite() || !feedforward.allFinite() || !std::isfinite(control_sign))
        {
            throw std::invalid_argument("Pitch LQR 输入含 NaN 或 Inf");
        }
        return feedforward - control_sign * parameters_.gain * (state - target);
    }

    const Parameters& parameters() const
    {
        require_configured();
        return parameters_;
    }

    /**
     * @brief 对相邻两个腿高工作点做线性插值
     * @param ratio 右侧工作点权重，范围 [0, 1]
     */
    static Parameters interpolate(const Parameters& lower, const Parameters& upper, double ratio)
    {
        if (!lower.all_finite() || !upper.all_finite() || !std::isfinite(ratio) ||
            ratio < 0.0 || ratio > 1.0)
        {
            throw std::invalid_argument("Pitch LQR 插值参数无效");
        }

        const double inverse = 1.0 - ratio;
        Parameters result;
        result.gain = inverse * lower.gain + ratio * upper.gain;
        result.x0 = inverse * lower.x0 + ratio * upper.x0;
        result.u0 = inverse * lower.u0 + ratio * upper.u0;
        return result;
    }

private:
    Parameters parameters_ {};
    bool configured_ {false};

    void require_configured() const
    {
        if (!configured_)
        {
            throw std::logic_error("Pitch LQR 尚未配置工作点参数");
        }
    }
};
