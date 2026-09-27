/**
 * @file ekf.hpp
 * @author qingyu
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
#include <functional>

namespace algorithm::filter {

/**
* @brief 扩展卡尔曼滤波：过程与观测模型都是非线性的，各自在当前估计点用雅可比矩阵做一阶线性化，再按标准卡尔曼递推更新状态与协方差
*
* @tparam StateDim 状态向量维数
* @tparam MeasureDim 观测向量维数
* @tparam Scalar 标量类型，默认 double
*/
template <std::size_t StateDim, std::size_t MeasureDim, typename Scalar = double>
class ExtendedKalmanFilter {
public:
    using State             = Eigen::Matrix<Scalar, StateDim, 1>;                       // 状态向量 x
    using Measurement       = Eigen::Matrix<Scalar, MeasureDim, 1>;                     // 观测向量 z
    using StateMatrix       = Eigen::Matrix<Scalar, StateDim, StateDim>;                // 状态方阵，用于协方差 P 与过程噪声 Q
    using MeasureMatrix     = Eigen::Matrix<Scalar, MeasureDim, MeasureDim>;            // 观测方阵，用于观测噪声 R
    using ObserveJacobian   = Eigen::Matrix<Scalar, MeasureDim, StateDim>;              // 观测雅可比 H = ∂h/∂x
    using KalmanGain        = Eigen::Matrix<Scalar, StateDim, MeasureDim>;              // 卡尔曼增益 K
    using ProcessModel      = std::function<State(const State&)>;                       // 过程模型 f(x)
    using MeasureModel      = std::function<Measurement(const State&)>;                 // 观测模型 h(x)

    void set_state(const State& x, const StateMatrix& p) {
        x_ = x;
        p_ = p;
    }

    void set_process_noise(const StateMatrix& q)        { q_ = q; }
    void set_measurement_noise(const MeasureMatrix& r)  { r_ = r; }

    /**
     * @brief 预测步：用过程模型传播状态，用过程雅可比传播协方差
     *
     * @param f 过程模型 f(x)
     * @param process_jacobian 过程雅可比 F = ∂f/∂x，在 x_ 处取值
     * @return const State& 预测后的状态
     *
     * @note f 与 F 是分开传的，一致性只能靠调用方保证：F 必须是在当前 x_ 处取的 ∂f/∂x。
     *       若用了上一拍或标称工作点的雅可比，滤波器不会报错，只会收敛变慢甚至发散
     */
    const State& predict(const ProcessModel& f, const StateMatrix& process_jacobian) {
        x_ = f(x_);
        p_ = process_jacobian * p_ * process_jacobian.transpose() + q_;
        return x_;
    }

    /**
     * @brief 预测步（带控制输入）：x = f(x) + B u，协方差传播与无输入版本相同
     *
     * @tparam ControlDim 控制向量维数。必须是 int，不能跟类模板一样写 std::size_t：
     *         Eigen::Matrix 的行列模板参数是 int，写 size_t 会推导出 int 与声明不匹配，
     *         导致这个重载永远匹配不上
     * @param f 过程模型 f(x)
     * @param process_jacobian 过程雅可比 F = ∂f/∂x，在 x_ 处取值
     * @param b 控制输入矩阵 B
     * @param u 控制向量 u
     * @return const State& 预测后的状态
     *
     * @note F 的要求同无输入版本：必须在当前 x_ 处取值
     */
    template <int ControlDim>
    const State& predict(const ProcessModel& f, const StateMatrix& process_jacobian,
                         const Eigen::Matrix<Scalar, StateDim, ControlDim>& b,
                         const Eigen::Matrix<Scalar, ControlDim, 1>& u) {
        x_ = f(x_) + b * u;
        p_ = process_jacobian * p_ * process_jacobian.transpose() + q_;
        return x_;
    }

    /**
     * @brief 更新步：用观测量、观测模型和观测雅可比修正状态与协方差
     *
     * @param z 观测向量
     * @param h 观测模型 h(x)
     * @param observe_jacobian 观测雅可比 H = ∂h/∂x，在 x_ 处取值
     * @return const State& 修正后的状态
     *
     * @note h 与 H 是分开传的，一致性只能靠调用方保证：H 必须是在当前 x_ 处取的 ∂h/∂x，
     *       理由同 predict
     */
    const State& correct(const Measurement& z, const MeasureModel& h,
                         const ObserveJacobian& observe_jacobian) {
        const Measurement y = z - h(x_);
        const MeasureMatrix s = observe_jacobian * p_ * observe_jacobian.transpose() + r_;
        // K = P Hᵀ S⁻¹。S 对称，转置后等价于 (S⁻¹ H P)ᵀ，用 ldlt 分解求解代替求逆
        const KalmanGain k = s.ldlt().solve(observe_jacobian * p_).transpose();
        x_ = x_ + k * y;
        // Joseph 形式：比 (I - K H) P 多一层 (I - K H)ᵀ 和 K R Kᵀ，长时间递推能保住 P 的对称性与正定性
        const StateMatrix i_kh = StateMatrix::Identity() - k * observe_jacobian;
        p_ = i_kh * p_ * i_kh.transpose() + k * r_ * k.transpose();
        return x_;
    }

    const State& state() const { return x_; }
    const StateMatrix& covariance() const { return p_; }

private:
    State           x_{State::Zero()};                          // 状态估计
    StateMatrix     p_{StateMatrix::Identity()};                // 状态估计协方差
    StateMatrix     q_{StateMatrix::Identity()};                // 过程噪声协方差
    MeasureMatrix   r_{MeasureMatrix::Identity()};              // 观测噪声协方差
};

} // namespace algorithm::filter
