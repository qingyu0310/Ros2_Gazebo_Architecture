/**
 * @file lqr_gain_schedule.hpp
 * @brief 加载、校验并按腿高插值 Pitch/Roll 离线 LQR 参数
 *
 * @note 增益是离线工具（tools/generate_wheel_leg_lqr）按几个腿高工作点各解一套出来的，
 *       这里负责把它们读进来、核对"是不是当前这台车当前这份模型解出来的"，再按腿高插值。
 *       校验不过一律抛，不降级 —— 拿错模型的 K 去控车，表现是"车在抖"，比启动就崩难查得多
 * @note 超出调度表范围时用最近端点（不外推）。"超范围拒绝继续伸缩"是量目标的活，
 *       调用方按 height_kinematics 的可达区间先夹一次，这里只保证不给出没根据的 K
 */
#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "pitch_lqr.hpp"
#include "roll_lqr.hpp"

/**
 * @brief 增益调度表：按腿高排好序的一串工作点，按高度插出 Pitch/Roll 两套参数
 */
class LqrGainSchedule
{
public:
    /**
     * @brief 一个工作点：腿高 + 该高度下解出来的 Pitch/Roll 参数（X0/U0/K 等）
     */
    struct Point
    {
        double height {0.0};                    // 该工作点的腿高（髋轴到轮轴的竖直距离）m
        PitchLqrController::Parameters pitch {}; // Pitch 子系统在该高度的参数
        RollLqrController::Parameters roll {};   // Roll 子系统在该高度的参数
    };

    /**
     * @brief 插值结果：同样两套参数，只是高度是插出来的
     */
    struct Parameters
    {
        PitchLqrController::Parameters pitch {};
        RollLqrController::Parameters roll {};
    };

    /**
     * @brief 建一张空表
     *
     * @note 空表能构造出来，但 at() 会抛 logic_error —— 正常路径是先经过下面两个构造器之一，
     *       这里留着只是为了能在成员里先声明、之后再赋值
     */
    LqrGainSchedule() = default;

    /**
     * @brief 直接从已解析的工作点建表
     *
     * @param points 工作点列表，顺序无所谓，构造时按腿高排序
     *
     * @throw std::invalid_argument 列表为空或含 NaN/Inf
     * @throw std::runtime_error 有两个工作点的腿高几乎相同
     *
     * @note 给纯 C++ 测试和其他参数来源用：不走文件，也不校验模型哈希
     */
    explicit LqrGainSchedule(std::vector<Point> points) : points_(std::move(points))
    {
        sort_and_validate();
    }

    /**
     * @brief 从增益文件建立调度表：逐个加载、校验，再按腿高排序
     *
     * @param paths 增益文件绝对路径，顺序无所谓，加载完会排序
     * @param dt 控制周期，必须跟增益文件里记的一致
     * @param wheel_radius 轮半径，必须跟增益文件里记的一致
     * @param robot_description 当前 URDF 文本，用来核对模型哈希
     *
     * @throw std::invalid_argument 列表为空、dt/轮半径非正时抛
     * @throw std::runtime_error robot_description 为空，或某个文件校验不过时抛
     */
    LqrGainSchedule(const std::vector<std::string>& paths, double dt, double wheel_radius, const std::string& robot_description)
    {
        if (paths.empty())
        {
            throw std::invalid_argument("增益文件列表为空");
        }
        if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(wheel_radius) || wheel_radius <= 0.0)
        {
            throw std::invalid_argument("增益校验使用的 dt 和轮半径必须为有限正数");
        }
        if (robot_description.empty())
        {
            throw std::runtime_error("robot_description 为空，无法校验增益对应的模型版本");
        }

        // 哈希两边同一套规则（见 normalized_xml）：离线工具对展开后的 URDF 算一次，
        // 这里对参数进来的 robot_description 算一次，不一样就说明模型换了、增益过期了
        const std::string model_hash = hash_string(robot_description);
        for (const auto& path : paths)
        {
            points_.push_back(load_point(path, dt, wheel_radius, model_hash));
        }
        sort_and_validate();
    }

    /**
     * @brief 取指定腿高处的参数：在相邻两个工作点之间线性插值
     *
     * @param height 腿高 m（髋轴到轮轴的竖直距离）
     * @return Parameters 插出来的 Pitch/Roll 参数
     *
     * @throw std::logic_error 调度表还没加载
     * @throw std::invalid_argument height 不是有限数
     *
     * @note 超出表范围时落在最近端点上（比值夹到 0/1），不外推 —— 外推出来的 K 没有依据
     */
    Parameters at(double height) const
    {
        if (points_.empty())
        {
            throw std::logic_error("增益调度表尚未加载");
        }
        if (!std::isfinite(height))
        {
            throw std::invalid_argument("增益调度高度不是有限数");
        }

        // 先定位夹住 height 的两个工作点：落在表下面/上面时两端重合，插值比例自然取 0 或 1
        const Point* lower = &points_.front();
        const Point* upper = lower;
        if (height >= points_.back().height)
        {
            lower = &points_.back();
            upper = lower;
        }
        else if (height > points_.front().height)
        {
            for (std::size_t i = 1; i < points_.size(); ++i)
            {
                if (height <= points_[i].height)
                {
                    lower = &points_[i - 1];
                    upper = &points_[i];
                    break;
                }
            }
        }

        const double span = upper->height - lower->height;
        const double ratio = span > 0.0 ? std::clamp((height - lower->height) / span, 0.0, 1.0) : 0.0;
        return {PitchLqrController::interpolate(lower->pitch, upper->pitch, ratio), 
                 RollLqrController ::interpolate(lower->roll,  upper->roll,  ratio)};
    }

    /**
     * @brief 取排序后的工作点表
     *
     * @return const std::vector<Point>& 按腿高升序的工作点，诊断和测试用
     */
    const std::vector<Point>& points() const
    {
        return points_;
    }

private:
    std::vector<Point> points_;

    /**
     * @brief 排序 + 自检：非有限值、重复高度都在这里拦掉
     *
     * @throw std::invalid_argument 表为空或含 NaN/Inf
     * @throw std::runtime_error 两个工作点的腿高几乎相同（差 < 1e-9），插值会出现除零/跳变
     */
    void sort_and_validate()
    {
        if (points_.empty())
        {
            throw std::invalid_argument("增益工作点为空");
        }
        for (const auto& point : points_)
        {
            if (!std::isfinite(point.height) || !point.pitch.all_finite() || !point.roll.all_finite())
            {
                throw std::invalid_argument("增益工作点含 NaN 或 Inf");
            }
        }
        std::sort(points_.begin(), points_.end(),
                  [](const Point& left, const Point& right) { return left.height < right.height; });
        for (std::size_t i = 1; i < points_.size(); ++i)
        {
            if (points_[i].height - points_[i - 1].height < 1e-9)
            {
                throw std::runtime_error("增益调度表存在重复高度工作点");
            }
        }
    }

    /**
     * @brief 校验 YAML 里的状态/输入顺序标签
     *
     * @param node layout.states 或 layout.inputs 那一段
     * @param expected 期望的名字序列（跟 PitchLqrController / RollLqrController 里的枚举一一对应）
     * @param name 出错信息里用的段名
     *
     * @throw std::runtime_error 长度不对或第 i 个名字不匹配时抛
     *
     * @note 顺序是接口的一部分：增益矩阵的行列就是按这个顺序排的，标签对不上说明文件是
     *       别的版本生成的（或者顺序被人改过），此时矩阵再"看"着正常也不能用
     */
    template<std::size_t Size>
    static void require_layout(const YAML::Node& node, const std::array<const char*, Size>& expected, const std::string& name)
    {
        if (!node.IsSequence() || node.size() != Size)
        {
            throw std::runtime_error(name + " 维度不是 " + std::to_string(Size));
        }
        for (std::size_t i = 0; i < Size; ++i)
        {
            if (node[i].as<std::string>() != expected[i])
            {
                throw std::runtime_error(name + " 第 " + std::to_string(i) + " 项不匹配：期望 " + expected[i]);
            }
        }
    }

    /**
     * @brief 读一个二维矩阵，行列数必须跟模板参数一致
     *
     * @throw std::runtime_error 行数或某一行列数不对时抛
     */
    template<int Rows, int Columns>
    static Eigen::Matrix<double, Rows, Columns> load_matrix(const YAML::Node& node, const std::string& name)
    {
        if (!node.IsSequence() || node.size() != static_cast<std::size_t>(Rows))
        {
            throw std::runtime_error(name + " 行数不是 " + std::to_string(Rows));
        }
        Eigen::Matrix<double, Rows, Columns> result;
        for (int row = 0; row < Rows; ++row)
        {
            if (!node[row].IsSequence() || node[row].size() != static_cast<std::size_t>(Columns))
            {
                throw std::runtime_error(name + " 第 " + std::to_string(row) + " 行列数不是 " +
                                         std::to_string(Columns));
            }
            for (int column = 0; column < Columns; ++column)
            {
                result(row, column) = node[row][column].as<double>();
            }
        }
        return result;
    }

    /**
     * @brief 读一个增益文件，逐项校验后变成 Point
     *
     * @param path 增益文件
     * @param dt 期望的控制周期
     * @param wheel_radius 期望的轮半径
     * @param model_hash 当前模型算出来的哈希
     *
     * @throw std::runtime_error 缺字段、顺序标签不符、dt/轮半径/模型哈希对不上、
     *        维度不对、含 NaN/Inf 时抛
     *
     * @note 读的字段：layout（顺序标签）、model.dt / wheel_radius / hash（版本核对）、
     *       operating_point.height / x0 / u0（工作点）、gain.k（Pitch、Roll）、
     *       roll.operating_point.roll_per_q（roll 对腿差模的灵敏度）、roll.continuous.a/b
     *       （Roll 的开环矩阵，目前的用途是诊断与后续复核）
     */
    static Point load_point(const std::string& path, double dt, double wheel_radius, const std::string& model_hash)
    {
        const YAML::Node root = YAML::LoadFile(path);
        const std::array<const char*, PitchLqrController::kStateDim> pitch_states{
                "s",         "s_dot",            "pitch",      "pitch_rate", 
                "q_h_common", "q_h_common_rate", "q_k_common", "q_k_common_rate"};

        const std::array<const char*, PitchLqrController::kInputDim> pitch_inputs{"tau_w_common", "tau_h_common",  "tau_k_common"};
        const std::array<const char*, RollLqrController::kStateDim>  roll_states {"q_h_diff",     "q_h_diff_rate", "q_k_diff", "q_k_diff_rate"};
        const std::array<const char*, RollLqrController::kInputDim>  roll_inputs {"tau_h_diff",   "tau_k_diff"};

        require_layout(root["layout"]["states"],         pitch_states, "layout.states");
        require_layout(root["layout"]["inputs"],         pitch_inputs, "layout.inputs");
        require_layout(root["roll"]["layout"]["states"], roll_states,  "roll.layout.states");
        require_layout(root["roll"]["layout"]["inputs"], roll_inputs,  "roll.layout.inputs");

        // 版本核对三件套：控制周期、轮半径、模型哈希。任何一个对不上都说明这份增益不是
        // 给当前这台车/这份模型解的，直接用等于拿错模型的 K
        if (std::abs(root["model"]["dt"].as<double>() - dt) > 1e-12)
        {
            throw std::runtime_error("增益 dt 与 chassis dt 不一致：" + path);
        }
        if (std::abs(root["model"]["wheel_radius"].as<double>() - wheel_radius) > 1e-12)
        {
            throw std::runtime_error("增益轮半径与 chassis 参数不一致：" + path);
        }

        const std::string expected_hash = root["model"]["hash"].as<std::string>();
        if (expected_hash != model_hash)
        {
            throw std::runtime_error("增益模型哈希 " + expected_hash + " 与当前 URDF " + model_hash + " 不一致，请重新生成增益");
        }

        Point point;
        if (!root["operating_point"]["height"])
        {
            throw std::runtime_error("增益文件缺少 operating_point.height：" + path);
        }
        point.height = root["operating_point"]["height"].as<double>();

        // Pitch：X0/U0 是工作点的偏差基准，K 是反馈增益
        const YAML::Node pitch_x0 = root["operating_point"]["x0"];
        const YAML::Node pitch_u0 = root["operating_point"]["u0"];
        if (!pitch_x0.IsSequence() || pitch_x0.size() != PitchLqrController::kStateDim || !pitch_u0.IsSequence() || pitch_u0.size() != PitchLqrController::kInputDim)
        {
            throw std::runtime_error("Pitch X0/U0 维度不对：" + path);
        }
        for (std::size_t i = 0; i < PitchLqrController::kStateDim; ++i)
        {
            point.pitch.x0(static_cast<Eigen::Index>(i)) = pitch_x0[i].as<double>();
        }
        for (std::size_t i = 0; i < PitchLqrController::kInputDim; ++i)
        {
            point.pitch.u0(static_cast<Eigen::Index>(i)) = pitch_u0[i].as<double>();
        }
        point.pitch.gain = load_matrix<PitchLqrController::kInputDim, PitchLqrController::kStateDim>(root["gain"]["k"], "gain.k");

        // Roll：4 状态 [q_h−, q̇_h−, q_k−, q̇_k−]，roll_per_q 是 roll 对腿差模的灵敏度（诊断用）
        const YAML::Node roll_x0    = root["roll"]["operating_point"]["x0"];
        const YAML::Node roll_u0    = root["roll"]["operating_point"]["u0"];
        const YAML::Node roll_per_q = root["roll"]["operating_point"]["roll_per_q"];

        if (!roll_x0.IsSequence() || roll_x0.size() != RollLqrController::kStateDim || !roll_u0.IsSequence() || 
             roll_u0.size() != RollLqrController::kInputDim || !roll_per_q.IsSequence() || roll_per_q.size() != RollLqrController::kInputDim)
        {
            throw std::runtime_error("Roll X0/U0/roll_per_q 维度不对：" + path);
        }
        for (std::size_t i = 0; i < RollLqrController::kStateDim; ++i)
        {
            point.roll.x0(static_cast<Eigen::Index>(i)) = roll_x0[i].as<double>();
        }
        for (std::size_t i = 0; i < RollLqrController::kInputDim; ++i)
        {
            point.roll.u0(static_cast<Eigen::Index>(i)) = roll_u0[i].as<double>();
            point.roll.roll_per_q(static_cast<Eigen::Index>(i)) = roll_per_q[i].as<double>();
        }
        point.roll.gain = load_matrix<RollLqrController::kInputDim, RollLqrController::kStateDim>(root["roll"]["gain"]["k"], "roll.gain.k");

        // 开环 A/B 也一起读进来核对（有限性），虽然运行时不直接用
        const auto roll_a = load_matrix<RollLqrController::kStateDim, RollLqrController::kStateDim>(root["roll"]["continuous"]["a"], "roll.continuous.a");
        const auto roll_b = load_matrix<RollLqrController::kStateDim, RollLqrController::kInputDim>(root["roll"]["continuous"]["b"], "roll.continuous.b");

        if (!std::isfinite(point.height) || !point.pitch.all_finite() || !point.roll.all_finite() || !roll_a.allFinite() || !roll_b.allFinite())
        {
            throw std::runtime_error("增益参数含 NaN 或 Inf：" + path);
        }
        return point;
    }

    /**
     * @brief 把 XML 抹成"只剩结构"的形式：去掉注释、去掉引号外的所有空白
     *
     * @throw std::runtime_error 注释没有闭合时抛
     *
     * @note 哈希必须两边算法一样：离线工具是对 xacro 展开出来的文本算的，运行期拿到的是
     *       参数里的 robot_description（同一份内容，但注释和缩进可能不一样），所以先把
     *       注释和引号外的空白都抹掉再算，否则同一个模型会算出两个哈希
     */
    static std::string normalized_xml(const std::string& xml)
    {
        std::string uncommented;
        std::size_t cursor = 0;
        while (cursor < xml.size())
        {
            const std::size_t comment = xml.find("<!--", cursor);
            if (comment == std::string::npos)
            {
                uncommented.append(xml, cursor, std::string::npos);
                break;
            }
            uncommented.append(xml, cursor, comment - cursor);
            const std::size_t end = xml.find("-->", comment + 4);
            if (end == std::string::npos)
            {
                throw std::runtime_error("robot_description 里的 XML 注释没有闭合");
            }
            cursor = end + 3;
        }

        // 去空白时要看着引号：属性值里的空格是内容的一部分，抹掉会改变哈希
        std::string result;
        char quote = '\0';
        for (const unsigned char c : uncommented)
        {
            if (quote == '\0' && (c == '\'' || c == '"'))
            {
                quote = static_cast<char>(c);
                result.push_back(static_cast<char>(c));
            }
            else if (quote != '\0' && c == static_cast<unsigned char>(quote))
            {
                quote = '\0';
                result.push_back(static_cast<char>(c));
            }
            else if (quote != '\0' || !std::isspace(c))
            {
                result.push_back(static_cast<char>(c));
            }
        }
        return result;
    }

    /**
     * @brief 模型版本哈希：FNV-1a 64，输出带 0x 前缀的十六进制
     *
     * @note 必须跟 tools/generate_wheel_leg_lqr 里那份逐字节一致，否则校验永远不过
     */
    static std::string hash_string(const std::string& xml)
    {
        std::uint64_t hash = 1469598103934665603ULL;
        for (const unsigned char c : normalized_xml(xml))
        {
            hash ^= c;
            hash *= 1099511628211ULL;
        }
        std::ostringstream out;
        out << "0x" << std::hex << hash;
        return out.str();
    }
};
