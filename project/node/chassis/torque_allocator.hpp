/**
 * @file torque_allocator.hpp
 * @author qingyu
 * @brief 六路力矩统一分配器：共模/差模恢复、软限位、成组限幅与变化率限制
 * @version 0.1
 * @date 2026-09-27
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

class TorqueAllocator
{
public:
    static constexpr std::size_t kActuatorCount = 6;
    static constexpr std::size_t kLegCount = 4;

    enum Actuator : std::size_t
    {
        kWheelLeft = 0,
        kWheelRight,
        kHipLeft,
        kHipRight,
        kKneeLeft,
        kKneeRight
    };

    struct Config
    {
        double dt {0.001};
        std::array<double, kActuatorCount> effort_limit {0.28, 0.28, 1.5, 1.5, 1.5, 1.5};
        std::array<double, kActuatorCount> effort_rate_limit {100.0, 100.0, 500.0, 500.0, 500.0, 500.0};
        std::array<double, kLegCount> hard_lower {-0.2618, -0.2618, -1.1345, -1.1345};
        std::array<double, kLegCount> hard_upper { 0.6109,  0.6109,  0.4363,  0.4363};
        std::array<double, kLegCount> soft_lower {-0.1618, -0.1618, -1.0345, -1.0345};
        std::array<double, kLegCount> soft_upper { 0.5109,  0.5109,  0.3363,  0.3363};
        std::array<double, kLegCount> soft_limit_kp {2.0, 2.0, 2.0, 2.0};
        std::array<double, kLegCount> soft_limit_kd {0.05, 0.05, 0.05, 0.05};
    };

    struct Input
    {
        double wheel_common {0.0};
        double wheel_diff {0.0};
        double hip_common {0.0};
        double hip_diff {0.0};
        double knee_common {0.0};
        double knee_diff {0.0};
    };

    struct Feedback
    {
        std::array<double, kLegCount> position {};
        std::array<double, kLegCount> velocity {};
    };

    struct Diagnostics
    {
        bool priority_limited {false};
        bool effort_limited {false};
        bool rate_limited {false};
        bool soft_limit_active {false};
        bool invalid_input {false};
        double effort_scale {1.0};
        double rate_scale {1.0};
        std::array<double, kLegCount> soft_limit_effort {};
    };

    struct Result
    {
        std::array<double, kActuatorCount> effort {};
        Diagnostics diagnostics {};
    };

    explicit TorqueAllocator(const Config& config) : config_(config)
    {
        validate_config();
    }

    void reset()
    {
        previous_effort_.fill(0.0);
    }

    const std::array<double, kActuatorCount>& previous_effort() const
    {
        return previous_effort_;
    }

    Result allocate(const Input& input, const Feedback& feedback)
    {
        Result result;
        if (!finite(input) || !finite(feedback))
        {
            result.diagnostics.invalid_input = true;
            reset();
            return result;
        }

        // U 中每个共模/差模都是单侧执行器量：左=共+差，右=共-差。
        allocate_pair(input.wheel_common, input.wheel_diff, kWheelLeft, kWheelRight,
                      result.effort, result.diagnostics);
        allocate_pair(input.hip_common, input.hip_diff, kHipLeft, kHipRight,
                      result.effort, result.diagnostics);
        allocate_pair(input.knee_common, input.knee_diff, kKneeLeft, kKneeRight,
                      result.effort, result.diagnostics);

        apply_soft_limits(feedback, result);
        scale_groups_to_effort_limits(result);
        scale_groups_to_rate_limits(result);

        previous_effort_ = result.effort;
        return result;
    }

private:
    Config config_;
    std::array<double, kActuatorCount> previous_effort_ {};

    static bool finite(const Input& input)
    {
        return std::isfinite(input.wheel_common) && std::isfinite(input.wheel_diff) &&
               std::isfinite(input.hip_common) && std::isfinite(input.hip_diff) &&
               std::isfinite(input.knee_common) && std::isfinite(input.knee_diff);
    }

    static bool finite(const Feedback& feedback)
    {
        for (std::size_t i = 0; i < kLegCount; ++i)
        {
            if (!std::isfinite(feedback.position[i]) || !std::isfinite(feedback.velocity[i]))
            {
                return false;
            }
        }
        return true;
    }

    void validate_config() const
    {
        if (!std::isfinite(config_.dt) || config_.dt <= 0.0)
        {
            throw std::invalid_argument("TorqueAllocator dt 必须大于 0");
        }
        for (std::size_t i = 0; i < kActuatorCount; ++i)
        {
            if (!std::isfinite(config_.effort_limit[i]) || config_.effort_limit[i] <= 0.0 ||
                !std::isfinite(config_.effort_rate_limit[i]) || config_.effort_rate_limit[i] <= 0.0)
            {
                throw std::invalid_argument("TorqueAllocator 力矩上限和变化率必须为有限正数");
            }
        }
        for (std::size_t i = 0; i < kLegCount; ++i)
        {
            if (!(config_.hard_lower[i] < config_.soft_lower[i] &&
                  config_.soft_lower[i] < config_.soft_upper[i] &&
                  config_.soft_upper[i] < config_.hard_upper[i]) ||
                config_.soft_limit_kp[i] < 0.0 || config_.soft_limit_kd[i] < 0.0)
            {
                throw std::invalid_argument("TorqueAllocator 软硬限位或恢复增益无效");
            }
        }
    }

    void allocate_pair(double common, double difference, std::size_t left, std::size_t right,
                       std::array<double, kActuatorCount>& effort, Diagnostics& diagnostics) const
    {
        // 平衡共模优先，差模只吃左右两侧都剩下的力矩区间。
        const double common_limit = std::min(config_.effort_limit[left], config_.effort_limit[right]);
        const double allocated_common = std::clamp(common, -common_limit, common_limit);
        const double difference_lower = std::max(-config_.effort_limit[left] - allocated_common,
                                                   allocated_common - config_.effort_limit[right]);
        const double difference_upper = std::min( config_.effort_limit[left] - allocated_common,
                                                   allocated_common + config_.effort_limit[right]);
        const double allocated_difference = std::clamp(difference, difference_lower, difference_upper);

        effort[left] = allocated_common + allocated_difference;
        effort[right] = allocated_common - allocated_difference;
        diagnostics.priority_limited = diagnostics.priority_limited ||
            std::abs(allocated_common - common) > 1e-12 ||
            std::abs(allocated_difference - difference) > 1e-12;
    }

    void apply_soft_limits(const Feedback& feedback, Result& result) const
    {
        constexpr std::array<std::size_t, kLegCount> actuator{kHipLeft, kHipRight, kKneeLeft, kKneeRight};
        for (std::size_t i = 0; i < kLegCount; ++i)
        {
            double recovery = 0.0;
            if (feedback.position[i] < config_.soft_lower[i])
            {
                recovery = config_.soft_limit_kp[i] * (config_.soft_lower[i] - feedback.position[i])
                         - config_.soft_limit_kd[i] * feedback.velocity[i];
            }
            else if (feedback.position[i] > config_.soft_upper[i])
            {
                recovery = config_.soft_limit_kp[i] * (config_.soft_upper[i] - feedback.position[i])
                         - config_.soft_limit_kd[i] * feedback.velocity[i];
            }
            result.diagnostics.soft_limit_effort[i] = recovery;
            result.diagnostics.soft_limit_active = result.diagnostics.soft_limit_active || std::abs(recovery) > 0.0;
            result.effort[actuator[i]] += recovery;
        }
    }

    void scale_groups_to_effort_limits(Result& result) const
    {
        for (const auto pair : {std::array<std::size_t, 2>{kWheelLeft, kWheelRight},
                                std::array<std::size_t, 2>{kHipLeft, kHipRight},
                                std::array<std::size_t, 2>{kKneeLeft, kKneeRight}})
        {
            double scale = 1.0;
            for (const std::size_t index : pair)
            {
                if (std::abs(result.effort[index]) > config_.effort_limit[index])
                {
                    scale = std::min(scale, config_.effort_limit[index] / std::abs(result.effort[index]));
                }
            }
            if (scale < 1.0)
            {
                result.effort[pair[0]] *= scale;
                result.effort[pair[1]] *= scale;
                result.diagnostics.effort_limited = true;
                result.diagnostics.effort_scale = std::min(result.diagnostics.effort_scale, scale);
            }
        }
    }

    void scale_groups_to_rate_limits(Result& result) const
    {
        for (const auto pair : {std::array<std::size_t, 2>{kWheelLeft, kWheelRight},
                                std::array<std::size_t, 2>{kHipLeft, kHipRight},
                                std::array<std::size_t, 2>{kKneeLeft, kKneeRight}})
        {
            double scale = 1.0;
            std::array<double, 2> delta{};
            for (std::size_t i = 0; i < pair.size(); ++i)
            {
                const std::size_t index = pair[i];
                delta[i] = result.effort[index] - previous_effort_[index];
                const double maximum_delta = config_.effort_rate_limit[index] * config_.dt;
                if (std::abs(delta[i]) > maximum_delta)
                {
                    scale = std::min(scale, maximum_delta / std::abs(delta[i]));
                }
            }
            if (scale < 1.0)
            {
                for (std::size_t i = 0; i < pair.size(); ++i)
                {
                    const std::size_t index = pair[i];
                    result.effort[index] = previous_effort_[index] + scale * delta[i];
                }
                result.diagnostics.rate_limited = true;
                result.diagnostics.rate_scale = std::min(result.diagnostics.rate_scale, scale);
            }
        }
    }
};
