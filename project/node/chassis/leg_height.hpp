/**
 * @file leg_height.hpp
 * @brief 对称腿高与髋膝共模角之间的运动学换算
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

class LegHeightKinematics
{
public:
    struct Config
    {
        double link_length {0.056};
        double nominal_link_angle {0.6981317008};
        double hip_lower {-0.1618};
        double hip_upper {0.5109};
        double knee_lower {-1.0345};
        double knee_upper {0.3363};
    };

    struct Reference
    {
        double height {0.0};
        double hip {0.0};
        double knee {0.0};
    };

    explicit LegHeightKinematics(const Config& config) : config_(config)
    {
        if (!std::isfinite(config_.link_length) || config_.link_length <= 0.0 ||
            !std::isfinite(config_.nominal_link_angle) ||
            !std::isfinite(config_.hip_lower) || !std::isfinite(config_.hip_upper) ||
            !std::isfinite(config_.knee_lower) || !std::isfinite(config_.knee_upper) ||
            !(config_.hip_lower < config_.hip_upper) || !(config_.knee_lower < config_.knee_upper))
        {
            throw std::invalid_argument("腿高运动学参数无效");
        }

        // 对称伸缩 q_k=-2q_h，同时满足髋、膝各自的软限位。
        hip_lower_ = std::max(config_.hip_lower, -0.5 * config_.knee_upper);
        hip_upper_ = std::min(config_.hip_upper, -0.5 * config_.knee_lower);
        if (!(hip_lower_ < hip_upper_) ||
            std::cos(config_.nominal_link_angle + hip_lower_) <= 0.0 ||
            std::cos(config_.nominal_link_angle + hip_upper_) <= 0.0)
        {
            throw std::invalid_argument("对称伸缩区间为空或腿高映射不单调");
        }
    }

    double minimum_height() const
    {
        return height_from_hip(hip_lower_);
    }

    double maximum_height() const
    {
        return height_from_hip(hip_upper_);
    }

    double nominal_height() const
    {
        return height_from_hip(0.0);
    }

    Reference from_height(double requested_height) const
    {
        if (!std::isfinite(requested_height))
        {
            throw std::invalid_argument("目标腿高必须是有限数");
        }
        const double height = std::clamp(requested_height, minimum_height(), maximum_height());
        const double sine = std::clamp(height / (2.0 * config_.link_length), -1.0, 1.0);
        const double hip = std::clamp(
            std::asin(sine) - config_.nominal_link_angle, hip_lower_, hip_upper_);
        return {height_from_hip(hip), hip, -2.0 * hip};
    }

    double height_from_hip(double hip) const
    {
        return 2.0 * config_.link_length * std::sin(config_.nominal_link_angle + hip);
    }

    static double move_towards(double current, double target, double maximum_rate, double dt)
    {
        if (!std::isfinite(current) || !std::isfinite(target) || !std::isfinite(maximum_rate) ||
            !std::isfinite(dt) || maximum_rate <= 0.0 || dt <= 0.0)
        {
            throw std::invalid_argument("腿高斜坡参数无效");
        }
        const double maximum_step = maximum_rate * dt;
        return current + std::clamp(target - current, -maximum_step, maximum_step);
    }

private:
    Config config_;
    double hip_lower_ {0.0};
    double hip_upper_ {0.0};
};
