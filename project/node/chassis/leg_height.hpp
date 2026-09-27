/**
 * @file leg_height.hpp
 * @author qingyu
 * @brief 对称腿高与髋膝共模角之间的运动学换算
 * @version 0.1
 * @date 2026-09-27
 *
 * @details
 * 这个文件不是 ROS 节点，也不计算关节力矩，只给 ChassisNode 提供三个纯运动学功能：
 *
 * 1. 根据髋、膝软限位计算对称伸缩时允许的身高范围；
 * 2. 把目标身高换算成 Pitch LQR 使用的髋、膝共模目标角；
 * 3. 限制身高参考每一拍的最大变化量，避免目标阶跃冲击平衡环。
 *
 * 单侧腿由两根等长连杆组成。标准姿态下两根杆关于竖直方向前后对称；为了让轮心在
 * 伸缩过程中继续位于髋轴正下方，髋、膝共模角必须满足：
 *
 *     q_k+ = -2 q_h+
 *
 * 设单根杆长为 L，零位杆与水平面的夹角为 alpha，则髋轴到轮轴的竖直距离为：
 *
 *     h = 2 L sin(alpha + q_h+)
 *
 * 本类只处理左右腿共同运动的“共模”坐标；左右腿高度差和 Roll 差模不在这里计算。
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

/**
 * @brief 对称腿高参考生成器
 *
 * @note 类内不保存随时间变化的状态。除构造时计算可达区间外，所有接口都是确定性的
 *       运动学换算；当前高度参考由 ChassisNode::height_reference_ 保存。
 */
class LegHeightKinematics
{
public:
    /**
     * @brief 腿几何和关节软限位
     *
     * @note 这里应传入软限位，而不是 URDF 的硬限位。ChassisNode 在构造本类前，已经用
     *       leg_soft_limit_margin 将硬限位向内收缩，给恢复力矩留出空间。
     */
    struct Config
    {
        double link_length        {0.056};          ///< L，单根大腿/小腿长度，m
        double nominal_link_angle {0.6981317008};   ///< alpha，零关节角时杆与水平面的夹角，rad
        double hip_lower          {-0.1618};        ///< 髋共模允许下限，rad
        double hip_upper          {0.5109};         ///< 髋共模允许上限，rad
        double knee_lower         {-1.0345};        ///< 膝共模允许下限，rad
        double knee_upper         {0.3363};         ///< 膝共模允许上限，rad
    };

    /**
     * @brief 某个目标身高对应的一组共模参考
     *
     * `hip`、`knee` 会写入 Pitch LQR 的 Xref；它们不是某一侧的差模角。左右腿最终目标相同，
     * Roll 控制器再在实体执行器力矩上叠加差模修正。
     */
    struct Reference
    {
        double height {0.0};   ///< 夹到可达区间并反算后的实际参考高度，m
        double hip    {0.0};   ///< q_h+，髋共模目标角，rad
        double knee   {0.0};   ///< q_k+，膝共模目标角，rad；始终等于 -2*hip
    };

    /**
     * @brief 校验几何参数，并求同时满足髋、膝软限位的髋共模区间
     *
     * 膝角由 q_k+=-2q_h+ 约束，因此膝软限位可以换算成髋角限制：
     *
     *     knee_lower <= -2 q_h+ <= knee_upper
     *     -knee_upper/2 <= q_h+ <= -knee_lower/2
     *
     * 最终区间是上述范围与髋自身软限位的交集。
     *
     * @param config 腿长、标准角和髋膝软限位
     * @throw std::invalid_argument 参数非有限、区间为空，或区间内高度映射不单调
     */
    explicit LegHeightKinematics(const Config& config) : config_(config)
    {
        if (!std::isfinite(config_.link_length) || config_.link_length <= 0.0 || !std::isfinite(config_.nominal_link_angle) ||
            !std::isfinite(config_.hip_lower)   || !std::isfinite(config_.hip_upper) || !std::isfinite(config_.knee_lower) ||
            !std::isfinite(config_.knee_upper)  || !(config_.hip_lower < config_.hip_upper) || !(config_.knee_lower < config_.knee_upper))
        {
            throw std::invalid_argument("腿高运动学参数无效");
        }

        // 求 q_h+ 自身区间与 q_k+=-2q_h+ 换算区间的交集。
        hip_lower_ = std::max(config_.hip_lower, -0.5 * config_.knee_upper);
        hip_upper_ = std::min(config_.hip_upper, -0.5 * config_.knee_lower);

        // dh/dq_h+ = 2L*cos(alpha+q_h+)。要求区间两端导数为正，使整个允许区间落在
        // sin() 的同一单调分支上，from_height() 才能用唯一的 asin() 反解。
        if (!(hip_lower_ < hip_upper_) || std::cos(config_.nominal_link_angle + hip_lower_) <= 0.0 || std::cos(config_.nominal_link_angle + hip_upper_) <= 0.0)
        {
            throw std::invalid_argument("对称伸缩区间为空或腿高映射不单调");
        }
    }

    /**
     * @brief 当前髋膝软限位允许的最小对称腿高
     * @return 髋轴到轮轴的竖直距离，m
     */
    double minimum_height() const
    {
        return height_from_hip(hip_lower_);
    }

    /**
     * @brief 当前髋膝软限位允许的最大对称腿高
     * @return 髋轴到轮轴的竖直距离，m
     */
    double maximum_height() const
    {
        return height_from_hip(hip_upper_);
    }

    /**
     * @brief URDF 零关节角对应的标准腿高
     *
     * q_h+=0、q_k+=0，因此 h0=2L*sin(alpha)。
     *
     * @return 标准腿高，m
     */
    double nominal_height() const
    {
        return height_from_hip(0.0);
    }

    /**
     * @brief 把目标身高反解为髋膝共模目标角
     *
     * 计算顺序：
     *
     * 1. 将请求高度夹到软限位可达范围；
     * 2. 由 q_h+=asin(h/(2L))-alpha 反解髋共模角；
     * 3. 由 q_k+=-2q_h+ 得到膝共模角；
     * 4. 用反解后的髋角重新正算高度，保证返回的三个量严格自洽。
     *
     * @param requested_height 请求的髋轴到轮轴竖直距离，m
     * @return 可直接写入 Pitch LQR 目标状态的高度、髋角和膝角
     * @throw std::invalid_argument 请求高度为 NaN 或 Inf
     */
    Reference from_height(double requested_height) const
    {
        if (!std::isfinite(requested_height))
        {
            throw std::invalid_argument("目标腿高必须是有限数");
        }
        const double height = std::clamp(requested_height, minimum_height(), maximum_height());

        // 理论上 height/(2L) 已在 [-1,1]；再次 clamp 是防浮点舍入让 asin() 产生 NaN。
        const double sine   = std::clamp(height / (2.0 * config_.link_length), -1.0, 1.0);
        const double hip    = std::clamp(std::asin(sine) - config_.nominal_link_angle, hip_lower_, hip_upper_);

        return {height_from_hip(hip), hip, -2.0 * hip};
    }

    /**
     * @brief 根据髋共模角正算对称腿高
     *
     * 调用者必须保证这确实是对称伸缩姿态，即膝共模满足 q_k+=-2q_h+。本函数只需要髋角，
     * 是因为在该约束下膝角已经不再是独立变量。
     *
     * @param hip q_h+，髋共模角，rad
     * @return h=2L*sin(alpha+q_h+)，m
     */
    double height_from_hip(double hip) const
    {
        return 2.0 * config_.link_length * std::sin(config_.nominal_link_angle + hip);
    }

    /**
     * @brief 一阶斜坡限制器：让当前身高以不超过指定速度靠近目标
     *
     * 每拍最大变化量为 maximum_rate*dt。若目标距离小于该值，本拍直接到达目标；否则只前进
     * 一个最大步长。该函数同时适用于长高和变矮。
     *
     * @param current 当前指令高度，m；不是传感器测得的实际高度
     * @param target 最终目标高度，m
     * @param maximum_rate 最大高度变化速度，m/s
     * @param dt 控制周期，s
     * @return 这一控制拍应使用的新指令高度，m
     * @throw std::invalid_argument 任一参数非有限，或 maximum_rate/dt 不为正
     */
    static double move_towards(double current, double target, double maximum_rate, double dt)
    {
        if (!std::isfinite(current) || !std::isfinite(target) || !std::isfinite(maximum_rate) || !std::isfinite(dt) || maximum_rate <= 0.0 || dt <= 0.0)
        {
            throw std::invalid_argument("腿高斜坡参数无效");
        }
        const double maximum_step = maximum_rate * dt;
        return current + std::clamp(target - current, -maximum_step, maximum_step);
    }

private:
    Config config_;              ///< 构造时保存的几何参数和原始软限位
    double hip_lower_ {0.0};     ///< 同时满足髋膝软限位后的 q_h+ 下限，rad
    double hip_upper_ {0.0};     ///< 同时满足髋膝软限位后的 q_h+ 上限，rad
};
