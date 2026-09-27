/**
 * @file torque_allocator.hpp
 * @author qingyu
 * @brief 六路力矩统一分配器：共模/差模恢复、软限位、成组限幅与变化率限制
 * @version 0.1
 * @date 2026-09-27
 *
 * @details
 * ChassisNode 的三个控制环不会直接给出六个实体电机的力矩，而是给出三对执行器的
 * 共模/差模力矩：
 *
 *     轮：tau_w+（Pitch LQR） + tau_w-（Yaw 外环）
 *     髋：tau_h+（Pitch LQR） + tau_h-（Roll LQR）
 *     膝：tau_k+（Pitch LQR） + tau_k-（Roll LQR）
 *
 * 本类负责把它们恢复成固定顺序的六路实体力矩：
 *
 *     [左轮, 右轮, 左髋, 右髋, 左膝, 右膝]
 *
 * 对每一对执行器都使用：
 *
 *     tau_left  = tau_common + tau_diff
 *     tau_right = tau_common - tau_diff
 *
 * 分配顺序为：共模优先恢复左右力矩 -> 叠加腿关节软限位恢复力矩 -> 左右成组力矩限幅
 * -> 左右成组变化率限制。类本身不发布 ROS 话题，ChassisNode 取走 Result 后再发布。
 *
 * @copyright Copyright (c) 2026
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

/**
 * @brief 六执行器力矩分配与输出保护
 *
 * @note “共模优先”意味着执行器余量不足时先尽量保留 Pitch/身高控制，Roll/Yaw 差模只使用
 *       左右两侧共同剩余的力矩空间。这是明确的控制优先级，不是普通的逐路 clamp。
 * @note 硬限位越界停机由 SafetySupervisor 负责；本类中的 hard_lower/hard_upper
 *       用于校验软限位确实位于硬限位内部。本类只在软限位区施加恢复力矩。
 */
class TorqueAllocator
{
public:
    static constexpr std::size_t kActuatorCount = 6;   ///< 两轮、两髋、两膝
    static constexpr std::size_t kLegCount = 4;        ///< 需要关节限位保护的两髋、两膝

    /**
     * @brief 六路输出数组的固定下标
     *
     * Result::effort、Config::effort_limit、Config::effort_rate_limit 都必须按这个顺序排列。
     */
    enum Actuator : std::size_t
    {
        kWheelLeft = 0,
        kWheelRight,
        kHipLeft,
        kHipRight,
        kKneeLeft,
        kKneeRight
    };

    /**
     * @brief 分配器配置
     *
     * 腿关节数组顺序固定为：
     *
     *     [左髋, 右髋, 左膝, 右膝]
     */
    struct Config
    {
        double dt {0.001};   ///< 分配周期，s；用于把 N·m/s 换算成单拍最大力矩变化

        ///< 六个实体执行器的绝对力矩上限，N·m；顺序见 Actuator
        std::array<double, kActuatorCount> effort_limit {0.28, 0.28, 1.5, 1.5, 1.5, 1.5};

        ///< 六个实体执行器的力矩变化率上限，N·m/s；顺序见 Actuator
        std::array<double, kActuatorCount> effort_rate_limit {100.0, 100.0, 500.0, 500.0, 500.0, 500.0};

        std::array<double, kLegCount> hard_lower    {-0.2618,   -0.2618, -1.1345, -1.1345};   ///< 腿关节硬下限，rad
        std::array<double, kLegCount> hard_upper    {0.6109,    0.6109,  0.4363,  0.4363};    ///< 腿关节硬上限，rad
        std::array<double, kLegCount> soft_lower    {-0.1618,   -0.1618, -1.0345, -1.0345};   ///< 开始施加恢复力矩的软下限，rad
        std::array<double, kLegCount> soft_upper    {0.5109,    0.5109,  0.3363,  0.3363};    ///< 开始施加恢复力矩的软上限，rad
        std::array<double, kLegCount> soft_limit_kp {2.0,       2.0,     2.0,     2.0};       ///< 软限位恢复刚度，N·m/rad
        std::array<double, kLegCount> soft_limit_kd {0.05,      0.05,    0.05,    0.05};      ///< 软限位恢复阻尼，N·m·s/rad
    };

    /**
     * @brief 一拍控制器输入：三对执行器的虚拟共模/差模力矩
     *
     * 所有量的单位都是 N·m。这里一个 common 表示“每侧执行器各出这么多”，恢复左右力矩时
     * 不再除以 2。例如 hip_common=0.4 表示无差模时左右髋各输出 0.4 N·m。
     */
    struct Input
    {
        double wheel_common {0.0};   ///< tau_w+，来自 Pitch LQR
        double wheel_diff   {0.0};   ///< tau_w-，来自偏航角速度外环
        double hip_common   {0.0};   ///< tau_h+，来自 Pitch/身高 LQR
        double hip_diff     {0.0};   ///< tau_h-，来自 Roll LQR
        double knee_common  {0.0};   ///< tau_k+，来自 Pitch/身高 LQR
        double knee_diff    {0.0};   ///< tau_k-，来自 Roll LQR
    };

    /**
     * @brief 软限位计算需要的四个腿关节反馈
     *
     * 数组顺序固定为 [左髋, 右髋, 左膝, 右膝]；位置单位 rad，速度单位 rad/s。
     */
    struct Feedback
    {
        std::array<double, kLegCount> position {};
        std::array<double, kLegCount> velocity {};
    };

    /**
     * @brief 本拍发生过哪些裁剪/保护
     *
     * 这些标志供 ChassisNode 打日志。它们只描述本拍分配过程，不改变下一拍控制器状态。
     */
    struct Diagnostics
    {
        bool priority_limited  {false};   ///< 共模或差模在 allocate_pair() 中被裁剪
        bool effort_limited    {false};   ///< 叠加软限位后触发成组绝对力矩缩放
        bool rate_limited      {false};   ///< 相对上一拍触发成组变化率缩放
        bool soft_limit_active {false};   ///< 至少一个腿关节处于软限位恢复区
        bool invalid_input     {false};   ///< 输入或腿反馈含 NaN/Inf，本拍输出全零
        double effort_scale    {1.0};     ///< 三对执行器中最小的绝对力矩缩放系数
        double rate_scale      {1.0};     ///< 三对执行器中最小的变化率缩放系数

        ///< 四个腿关节本拍各自叠加的软限位恢复力矩，N·m；顺序同 Feedback
        std::array<double, kLegCount> soft_limit_effort {};
    };

    /**
     * @brief 一拍分配结果
     */
    struct Result
    {
        std::array<double, kActuatorCount> effort {};   ///< 六路最终力矩，N·m；顺序见 Actuator
        Diagnostics diagnostics {};                     ///< 本拍裁剪和保护信息
    };

    /**
     * @brief 保存配置并在启动阶段拒绝无效限位
     * @param config 六路执行器和四个腿关节的限制参数
     * @throw std::invalid_argument dt、限幅或软硬限位关系无效
     */
    explicit TorqueAllocator(const Config& config) : config_(config)
    {
        validate_config();
    }

    /**
     * @brief 清空变化率限制器的历史输出
     *
     * 安全停机和重新接管时调用。重置后下一拍从 0 N·m 按变化率限制重新爬升，避免恢复瞬间跳变。
     */
    void reset()
    {
        previous_effort_.fill(0.0);
    }

    /**
     * @brief 读取上一拍已经实际分配出去的六路力矩
     * @return 固定顺序 [左轮, 右轮, 左髋, 右髋, 左膝, 右膝]
     */
    const std::array<double, kActuatorCount>& previous_effort() const
    {
        return previous_effort_;
    }

    /**
     * @brief 完成一拍六路力矩分配
     *
     * 固定处理顺序：
     *
     * 1. 检查虚拟输入和腿反馈是否有限；
     * 2. 对轮、髋、膝分别执行共模优先的左右恢复；
     * 3. 给四个腿关节叠加软限位恢复力矩；
     * 4. 对轮、髋、膝三组分别执行绝对力矩成组缩放；
     * 5. 相对上一拍执行变化率成组缩放；
     * 6. 保存最终输出，供下一拍变化率限制使用。
     *
     * @param input Pitch、Roll、Yaw 控制器给出的虚拟力矩
     * @param feedback 四个腿关节的当前位置和速度
     * @return 六路安全力矩以及本拍诊断
     */
    Result allocate(const Input& input, const Feedback& feedback)
    {
        Result result;
        if (!finite(input) || !finite(feedback))
        {
            result.diagnostics.invalid_input = true;
            reset();
            return result;
        }

        // U 中每个共模/差模都是单侧执行器量：左=共+差，右=共-差，不需要再除以 2。
        allocate_pair(input.wheel_common, input.wheel_diff, kWheelLeft, kWheelRight, result.effort, result.diagnostics);
        allocate_pair(input.hip_common,   input.hip_diff,   kHipLeft,   kHipRight,   result.effort, result.diagnostics);
        allocate_pair(input.knee_common,  input.knee_diff,  kKneeLeft,  kKneeRight,  result.effort, result.diagnostics);

        apply_soft_limits(feedback, result);
        scale_groups_to_effort_limits(result);
        scale_groups_to_rate_limits(result);

        previous_effort_ = result.effort;
        return result;
    }

private:
    Config config_;   ///< 构造后固定的限幅、软限位和控制周期

    ///< 上一拍最终输出；不是 LQR 原始请求，用于对真正发出的力矩做变化率限制
    std::array<double, kActuatorCount> previous_effort_ {};

    /**
     * @brief 六个虚拟共模/差模力矩是不是都是有限数
     *
     * @param input 这一拍的虚拟力矩
     * @return bool 全有限返回 true，有一个 NaN/Inf 就 false
     *
     * @note 分配之前先过这一关：NaN 一旦混进力矩，后面的限幅和变化率比较全都失效
     *       （比较 NaN 恒为 false），会一路把 NaN 发到电机上
     */
    static bool finite(const Input& input)
    {
        return std::isfinite(input.wheel_common) && std::isfinite(input.wheel_diff) &&
               std::isfinite(input.hip_common)   && std::isfinite(input.hip_diff)   &&
               std::isfinite(input.knee_common)  && std::isfinite(input.knee_diff);
    }

    /**
     * @brief 四个腿关节的位置和速度是不是都是有限数
     *
     * @param feedback 这一拍的腿关节反馈，顺序 [左髋, 右髋, 左膝, 右膝]
     * @return bool 全有限返回 true
     *
     * @note 软限位恢复力矩是按位置和速度算的，这两个数组不干净就算不出有意义的值
     */
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

    /**
     * @brief 检查配置中的数值、正值约束和软硬限位嵌套关系
     *
     * 每个腿关节必须满足：hard_lower < soft_lower < soft_upper < hard_upper。
     */
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

    /**
     * @brief 对一对左右执行器执行“共模优先”的力矩恢复
     *
     * 先将 common 夹到两侧都能承受的范围，再由实体力矩约束：
     *
     *     -L_left  <= common + diff <= L_left
     *     -L_right <= common - diff <= L_right
     *
     * 联立求出 diff 的合法上下界。这样无论请求多大，恢复后的左右力矩都不会在这一阶段越限，
     * 且执行器余量不足时先牺牲差模。
     *
     * @param common 请求的单侧共模力矩，N·m
     * @param difference 请求的单侧差模力矩，N·m
     * @param left 左执行器在六路数组中的下标
     * @param right 右执行器在六路数组中的下标
     * @param effort 写入恢复后的左右实体力矩
     * @param diagnostics 若发生裁剪则置 priority_limited
     */
    void allocate_pair(double common, double difference, std::size_t left, std::size_t right,
                       std::array<double, kActuatorCount>& effort, Diagnostics& diagnostics) const
    {
        // 平衡共模优先，差模只使用左右两侧都剩下的力矩区间。
        const double common_limit     = std::min(config_.effort_limit[left], config_.effort_limit[right]);
        const double allocated_common = std::clamp(common, -common_limit, common_limit);
        const double difference_lower = std::max(-config_.effort_limit[left] - allocated_common, allocated_common - config_.effort_limit[right]);
        const double difference_upper = std::min( config_.effort_limit[left] - allocated_common, allocated_common + config_.effort_limit[right]);
        const double allocated_difference = std::clamp(difference, difference_lower, difference_upper);

        effort[left]  = allocated_common + allocated_difference;
        effort[right] = allocated_common - allocated_difference;
        diagnostics.priority_limited = diagnostics.priority_limited || std::abs(allocated_common - common) > 1e-12 || std::abs(allocated_difference - difference) > 1e-12;
    }

    /**
     * @brief 为越过软限位的髋膝关节叠加指向安全区的 PD 恢复力矩
     *
     * 低于软下限时恢复力矩为正，高于软上限时恢复力矩为负；阻尼项始终抑制继续向外运动。
     * 此步可能让前面已满足限制的实体力矩再次超限，所以后面必须再执行绝对力矩成组缩放。
     */
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

    /**
     * @brief 对轮、髋、膝三对执行器分别做统一比例的绝对力矩缩放
     *
     * 若一对中的任一侧越限，左右两侧同时乘同一个 scale。这样不会像逐路 clamp 那样任意破坏
     * 左右力矩比例。三对执行器互不共用缩放系数。
     */
    void scale_groups_to_effort_limits(Result& result) const
    {
        for (const auto pair : {std::array<std::size_t, 2>{kWheelLeft, kWheelRight},
                                                               std::array<std::size_t, 2>{kHipLeft,   kHipRight},
                                                               std::array<std::size_t, 2>{kKneeLeft,  kKneeRight}})
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

    /**
     * @brief 对轮、髋、膝三对执行器分别限制一拍内的力矩变化
     *
     * 对每一对计算从 previous_effort_ 到本拍目标的 delta。只要任一侧超过
     * effort_rate_limit*dt，左右两侧就沿各自 delta 同时前进相同的比例，保持成组动态一致。
     */
    void scale_groups_to_rate_limits(Result& result) const
    {
        for (const auto pair : {std::array<std::size_t, 2>{kWheelLeft, kWheelRight},
                                                               std::array<std::size_t, 2>{kHipLeft,   kHipRight},
                                                               std::array<std::size_t, 2>{kKneeLeft,  kKneeRight}})
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
