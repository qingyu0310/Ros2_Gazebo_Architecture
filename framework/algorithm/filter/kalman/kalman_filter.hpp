/**
 * @file kalman_filter.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-09-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <Eigen/Dense>
#include <cstddef>

namespace algorithm::filter {

/**
* @brief 线性卡尔曼滤波：状态方程与观测方程都是线性的，按预测/更新两步递推状态估计与协方差
*
* @tparam StateDim 状态向量维数
* @tparam MeasureDim 观测向量维数
* @tparam Scalar 标量类型，默认 double
*/
template <std::size_t StateDim, std::size_t MeasureDim, typename Scalar = double>
class KalmanFilter 
{
public:
    using State         = Eigen::Matrix<Scalar, StateDim, 1>;             // 状态向量 x
    using Measurement   = Eigen::Matrix<Scalar, MeasureDim, 1>;           // 观测向量 z
    using StateMatrix   = Eigen::Matrix<Scalar, StateDim, StateDim>;      // 状态方阵，用于协方差 P 与过程噪声 Q
    using MeasureMatrix = Eigen::Matrix<Scalar, MeasureDim, MeasureDim>;  // 观测方阵，用于观测噪声 R
    using ObserveMatrix = Eigen::Matrix<Scalar, MeasureDim, StateDim>;    // 观测矩阵 H
    using KalmanGain    = Eigen::Matrix<Scalar, StateDim, MeasureDim>;    // 卡尔曼增益 K

    void set_state(const State& x, const StateMatrix& p) {
        x_ = x;
        p_ = p;
    }

    void set_process_noise(const StateMatrix& q) { q_ = q; }
    void set_measurement_noise(const MeasureMatrix& r) { r_ = r; }

    /**
     * @brief 预测步（无控制输入）：用状态转移矩阵传播状态与协方差
     *
     * @param f 状态转移矩阵 F
     * @return const State& 预测后的状态
     */
    const State& predict(const StateMatrix& f) {
        x_ = f * x_;
        p_ = f * p_ * f.transpose() + q_;
        return x_;
    }

    /**
     * @brief 预测步（带控制输入）：x = F x + B u，协方差传播与无输入版本相同
     *
     * @tparam ControlDim 控制向量维数
     * @param f 状态转移矩阵 F
     * @param b 控制输入矩阵 B
     * @param u 控制向量 u
     * @return const State& 预测后的状态
     *
     * @note ControlDim 只能写 int，这是"必须"的例外：Eigen::Matrix 的行列模板参数就是 int，
     *       既换不成定宽类型也换不成无符号类型 —— 写成 std::size_t 会从矩阵推导出 int 与声明
     *       不匹配，这个重载永远匹配不上
     */
    template <int ControlDim>
    const State& predict(const StateMatrix& f, const Eigen::Matrix<Scalar, StateDim, ControlDim>& b, const Eigen::Matrix<Scalar, ControlDim, 1>& u) {
        x_ = f * x_ + b * u;
        p_ = f * p_ * f.transpose() + q_;
        return x_;
    }

    /**
     * @brief 更新步：用观测量、观测矩阵和卡尔曼增益修正状态与协方差
     *
     * @param z 观测向量
     * @param h 观测矩阵 H
     * @return const State& 修正后的状态
     */
    const State& correct(const Measurement& z, const ObserveMatrix& h) 
    {
        const Measurement y = z - h * x_;
        const MeasureMatrix s = h * p_ * h.transpose() + r_;
        // K = P Hᵀ S⁻¹。S 对称，转置后等价于 (S⁻¹ H P)ᵀ，用 ldlt 分解求解代替求逆
        const KalmanGain k = s.ldlt().solve(h * p_).transpose();
        x_ = x_ + k * y;
        // Joseph 形式：比 (I - K H) P 多一层 (I - K H)ᵀ 和 K R Kᵀ，长时间递推能保住 P 的对称性与正定性
        const StateMatrix i_kh = StateMatrix::Identity() - k * h;
        p_ = i_kh * p_ * i_kh.transpose() + k * r_ * k.transpose();
        return x_;
    }

    const State& state() const { return x_; }
    const StateMatrix& covariance() const { return p_; }

private:
    State           x_{State::Zero()};                  // 状态估计
    StateMatrix     p_{StateMatrix::Identity()};        // 状态估计协方差
    StateMatrix     q_{StateMatrix::Identity()};        // 过程噪声协方差
    MeasureMatrix   r_{MeasureMatrix::Identity()};      // 观测噪声协方差
};

} // namespace algorithm::filter
