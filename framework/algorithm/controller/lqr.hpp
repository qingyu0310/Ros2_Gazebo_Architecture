/**
 * @file lqr.hpp
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
#include <stdexcept>

namespace algorithm::controller {

/**
* @brief 线性二次型调节器：对离散状态方程 x[k+1] = A x[k] + B u[k]，求使二次代价 sum(x'Qx + u'Ru) 最小的最优反馈增益 K，控制律为 u = -K (x - target)
*
* @tparam StateDim 状态向量维数
* @tparam ControlDim 控制向量维数
* @tparam Scalar 标量类型，默认 double
*/
template <std::size_t StateDim, std::size_t ControlDim, typename Scalar = double>
class Lqr 
{
public:
    using State         = Eigen::Matrix<Scalar, StateDim, 1>;             // 状态向量 x
    using Control       = Eigen::Matrix<Scalar, ControlDim, 1>;           // 控制向量 u
    using StateMatrix   = Eigen::Matrix<Scalar, StateDim, StateDim>;      // 状态方阵，用于 A / Q / P
    using ControlMatrix = Eigen::Matrix<Scalar, ControlDim, ControlDim>;  // 控制方阵，用于 R / S
    using InputMatrix   = Eigen::Matrix<Scalar, StateDim, ControlDim>;    // 输入矩阵 B
    using GainMatrix    = Eigen::Matrix<Scalar, ControlDim, StateDim>;    // 反馈增益 K

    Lqr() = default;

    /**
     * @brief 构造时就地配置系统矩阵与权重，求解仍需另调 solve()
     *
     * @param a 状态矩阵 A
     * @param b 输入矩阵 B
     * @param q 状态权重矩阵 Q，对称半正定
     * @param r 控制权重矩阵 R，对称正定
     */
    Lqr(const StateMatrix& a, const InputMatrix& b, const StateMatrix& q, const ControlMatrix& r)
    {
        configure(a, b, q, r);
    }

    /**
     * @brief 设置系统矩阵与权重，并把已求解标志清掉
     *
     * @param a 状态矩阵 A
     * @param b 输入矩阵 B
     * @param q 状态权重矩阵 Q，对称半正定
     * @param r 控制权重矩阵 R，对称正定
     */
    void configure(const StateMatrix& a, const InputMatrix& b, const StateMatrix& q, const ControlMatrix& r)
    {
        a_ = a;
        b_ = b;
        q_ = q;
        r_ = r;
        solved_ = false;
    }

    /**
     * @brief 迭代求解离散代数黎卡提方程，收敛后缓存最优反馈增益
     *
     * @param max_iterations 最大迭代次数，用尽则拿最后一次的 P 算增益
     * @param tolerance 收敛判据：相邻两次 P 的范数差小于它即认为收敛
     * @return const GainMatrix& 最优反馈增益 K
     */
    const GainMatrix& solve(std::size_t max_iterations = 100, Scalar tolerance = static_cast<Scalar>(1e-9))
    {
        StateMatrix p = q_;

        for (std::size_t i = 0; i < max_iterations; ++i) 
        {
            const ControlMatrix s      = r_ + b_.transpose() * p * b_;
            const GainMatrix    k      = s.ldlt().solve(b_.transpose() * p * a_);
            const StateMatrix   next_p = q_ + a_.transpose() * p * (a_ - b_ * k);

            if ((next_p - p).norm() < tolerance) {
                p_ = next_p;
                k_ = k;
                solved_ = true;
                return k_;
            }
            p = next_p;
        }

        p_ = p;
        const ControlMatrix s = r_ + b_.transpose() * p_ * b_;
        k_ = s.ldlt().solve(b_.transpose() * p_ * a_);
        solved_ = true;
        return k_;
    }

    /**
     * @brief 按控制律 u = -K (x - target) 计算控制量；未先 solve() 时抛 std::logic_error
     *
     * @param state 当前状态
     * @param target 目标状态，默认全零
     * @return Control 控制向量 u
     */
    Control update(const State& state, const State& target = State::Zero()) const
    {
        if (!solved_) {
            throw std::logic_error("LQR gain has not been solved");
        }
        return -k_ * (state - target);
    }

    const GainMatrix&   gain()      const { return k_; }
    const StateMatrix&  riccati()   const { return p_; }
    bool                solved()    const { return solved_; }

private:
    StateMatrix     a_{StateMatrix::Identity()};    // 系统矩阵 A
    InputMatrix     b_{InputMatrix::Zero()};        // 输入矩阵 B
    StateMatrix     q_{StateMatrix::Identity()};    // 状态权重 Q
    ControlMatrix   r_{ControlMatrix::Identity()};  // 控制权重 R
    StateMatrix     p_{StateMatrix::Zero()};        // 黎卡提方程的解 P
    GainMatrix      k_{GainMatrix::Zero()};         // 反馈增益 K
    
    bool            solved_{false};                 // 增益是否已求解
};

} // namespace algorithm::controller
