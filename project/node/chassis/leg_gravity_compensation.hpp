/**
 * @file leg_gravity_compensation.hpp
 * @brief 根据当前对称腿姿计算单侧髋膝静态重力前馈
 *
 * @note 这是 U0 的运行时版本：离线增益文件里那个 U0 只在生成它的腿高上成立，这里按**当前**
 *       腿姿（共模髋膝角）现算，所以调高调矮时前馈跟着走，不用等重新生成增益
 * @note 模型假设（跟离线生成器一致）：整车重量两条腿平分、地面法向力竖直向上且作用在
 *       轮心正下方；关节以外的每个竖直力对关节的力矩求和取反号，就是关节该出的力矩。
 *       标准腿姿下是 胯 -0.0147 / 膝 -0.2567 N·m —— 跟腿姿保持器那对一致
 */
#pragma once

#include <cmath>
#include <stdexcept>
#include <utility>

#include "chassis_types.hpp"

/**
 * @brief 单侧腿的重力前馈：给定共模髋膝角，算出这条腿的髋、膝各该出多大力矩
 *
 * @note 只算"顶住自重"这一份，不含反馈：反馈（K·(X-X0)）是 Pitch/Roll 控制器那边的事
 */
class LegGravityCompensation
{
public:
    /**
     * @brief 建一个未配置的实例
     *
     * @note 未配置时调用 effort() 会抛 logic_error；正常路径是用下面那个构造器
     */
    LegGravityCompensation() = default;

    /**
     * @brief 用模型参数和腿高参数（几何 + 质量）建实例
     *
     * @param model 模型参数，用其中的 body_mass 和 gravity
     * @param height 腿高参数，用其中的 link_length / link_angle / rod_mass / joint_mass / wheel_mass
     *
     * @throw std::invalid_argument 质量、重力或杆长非正（或非有限）时抛
     *
     * @note 这几个数必须跟 URDF 一份数：对不上不报错，只是前馈差一点、稳态误差大一点
     */
    LegGravityCompensation(const ModelParams& model, const HeightParams& height) : model_(model), height_(height)
    {
        if (!std::isfinite(model_.body_mass)    || model_.body_mass <= 0.0    || !std::isfinite(model_.gravity)     || model_.gravity <= 0.0    ||
            !std::isfinite(height_.link_length) || height_.link_length <= 0.0 || !std::isfinite(height_.rod_mass)   || height_.rod_mass <= 0.0  ||
            !std::isfinite(height_.joint_mass)  || height_.joint_mass <= 0.0  || !std::isfinite(height_.wheel_mass) || height_.wheel_mass <= 0.0)
        {
            throw std::invalid_argument("腿重力前馈参数无效");
        }
        configured_ = true;
    }

    /**
     * @brief 算这个腿姿下的单侧重力前馈
     *
     * @param hip 当前髋角 rad（共模）
     * @param knee 当前膝角 rad（共模）
     * @return std::pair<double, double> {单侧髋前馈, 单侧膝前馈}，单位 N·m
     *
     * @throw std::logic_error 没配置过，或者输入不是有限数
     *
     * @note 位置从矢状面的腿链上取：大腿远端在 (span, -drop)、小腿远端再往回 (-span, -drop)，
     *       标准腿姿下轮心正好落在胯轴正下方 —— 所以地面对胯几乎不产生力矩（只剩连杆自重），
     *       而膝那边整车重量在两腿上的分量压在整个 span 上，这就是膝那份大得多的原因
     */
    std::pair<double, double> effort(double hip, double knee) const
    {
        if (!configured_ || !std::isfinite(hip) || !std::isfinite(knee))
        {
            throw std::logic_error("腿重力前馈未配置或输入无效");
        }

        // 腿链上各点相对髋轴的横向位置（竖直力的力矩只看横向偏移，z 不进来）
        const double span    = height_.link_length * std::cos(height_.link_angle);
        const double drop    = height_.link_length * std::sin(height_.link_angle);
        const double knee_x  = rotated_x(hip, span, -drop);
        const double wheel_x = knee_x + rotated_x(hip + knee, -span, -drop);
        const double thigh_com_x = rotated_x(hip, 0.5 * span, -0.5 * drop);
        const double calf_com_x  = knee_x + rotated_x(hip + knee, -0.5 * span, -0.5 * drop);

        // 地面对这条腿的法向力：整车重量（车身 + 两个轮子）两腿平分
        const double normal_force = 0.5 * (model_.body_mass + 2.0 * height_.wheel_mass) * model_.gravity;
        const double rod_weight   = -height_.rod_mass * model_.gravity;
        const double wheel_weight = -height_.wheel_mass * model_.gravity;
        const double joint_weight = -height_.joint_mass * model_.gravity;

        // 竖直力 force_z 在横向偏移 offset_x 处，对关节产生的绕 +y 力矩
        const auto moment = [](double offset_x, double force_z) { return -offset_x * force_z; };

        // 膝：远端是小腿 + 轮子 + 地面法向力（大腿和膝上关节块在膝这边算近端，不算）
        const double knee_external = moment(wheel_x - knee_x, normal_force) + moment(calf_com_x - knee_x, rod_weight) 
                                   + moment(wheel_x - knee_x, wheel_weight);

        // 胯：整条腿都在远端（大腿、膝上关节块、小腿、轮子）+ 地面法向力
        const double hip_external  = moment(wheel_x, normal_force) + moment(thigh_com_x, rod_weight)
                                   + moment(knee_x,  joint_weight) + moment(calf_com_x,  rod_weight)
                                   + moment(wheel_x, wheel_weight);
        // 反号 = 关节该出的力矩
        return {-hip_external, -knee_external};
    }

private:
    bool configured_ {false};   // 没配置过就调 effort() 会抛，别拿 0 当前馈
    
    ModelParams  model_ {};      // 只用 body_mass 和 gravity
    HeightParams height_ {};    // 只用几何（link_length / link_angle）和质量（rod/joint/wheel_mass）

    /**
     * @brief 绕 +y 转 angle 之后，局部点 (x, z) 相对关节的横向偏移
     *
     * @param angle 关节角 rad
     * @param x 点相对关节的横向位置 m
     * @param z 点相对关节的竖直位置 m（力矩不看它，但旋转要它）
     * @return double 转到世界方向后的 x 偏移 m
     */
    static double rotated_x(double angle, double x, double z)
    {
        return x * std::cos(angle) + z * std::sin(angle);
    }
};
