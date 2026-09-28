/**
 * @file generate_wheel_leg_lqr.cpp
 * @author qingyu
 * @brief 离线工具：从展开的 URDF 建平面共模模型，线性化 A/B，解 K，写增益文件
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 * 用法（在仓库根目录跑）：
 *   ros2 run project generate_wheel_leg_lqr [xacro或urdf] [输出yaml] [参数yaml] [腿高m]
 *   默认：project/robot/models/bodys/wheel_leg_robot.xacro
 *         project/params/leg_gain/nominal.yaml
 *         project/params/chassis.yaml   （从这里取 q_pitch / r_pitch）
 *
 * 模型与公式的依据：docs/轮腿机器人平面模型参数与公式.md（那边写死了口径，改这里先改那边）
 *
 * 为什么是离线工具（文档 §4.4）：A/B 必须从 URDF 现生成、带检查、带模型版本，不许在 yaml 里
 * 手填一组没有来路的矩阵。运行时只加载这份增益，做一次矩阵乘。
 *
 * 跑完会打：模型摘要、工作点 X0/U0、A/B、K、特征值、九项检查。
 * 检查不过就以非零退出码结束，别把没验证过的增益当成能用的。
 */

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include "framework/algorithm/controller/lqr.hpp"

namespace {

// ---- 顺序定义：状态、输入、广义坐标全写在这里 ----
constexpr std::size_t kStateDim   = 8;
constexpr std::size_t kInputDim   = 3;
constexpr std::size_t kGeneralDim = 4;

constexpr std::size_t kS  = 0;   // 广义坐标：轮心位移
constexpr std::size_t kTh = 1;   // 机身俯仰
constexpr std::size_t kQh = 2;   // 髋（共模）
constexpr std::size_t kQk = 3;   // 膝（共模）

constexpr std::size_t kTauWheel = 0;
constexpr std::size_t kTauHip   = 1;
constexpr std::size_t kTauKnee  = 2;

constexpr int kBody  = 0;
constexpr int kThigh = 1;
constexpr int kCalf  = 2;
constexpr int kWheel = 3;

using GeneralVector  = Eigen::Matrix<double, kGeneralDim, 1>;
using StateVector    = Eigen::Matrix<double, kStateDim, 1>;
using StateMatrix    = Eigen::Matrix<double, kStateDim, kStateDim>;
using InputMatrix    = Eigen::Matrix<double, kStateDim, kInputDim>;
using ControlMatrix  = Eigen::Matrix<double, kInputDim, kInputDim>;
using GeneralMatrix  = Eigen::Matrix<double, kGeneralDim, kGeneralDim>;
using GeneralInput   = Eigen::Matrix<double, kGeneralDim, kInputDim>;
using Jacobian       = Eigen::Matrix<double, 2, kGeneralDim>;
using RateRow        = Eigen::Matrix<double, 1, kGeneralDim>;

const std::array<const char*, kStateDim> kStateNames{
    "s", "s_dot", "pitch", "pitch_rate", "q_h_common", "q_h_common_rate", "q_k_common", "q_k_common_rate"};
const std::array<const char*, kInputDim> kInputNames{"tau_w_common", "tau_h_common", "tau_k_common"};

// ===========================================================================
// URDF 读取
// ===========================================================================

struct LinkInfo
{
    std::string name;
    double mass {0.0};
    double ixx {0.0};
    double iyy {0.0};
    double izz {0.0};
    double com_x {0.0};
    double com_z {0.0};
    double inertial_roll {0.0};       // 惯量 origin 的 rpy，校验口径用
    double inertial_yaw {0.0};
    double radius {0.0};              // 只有圆柱 link（轮子）有
};

struct JointInfo
{
    std::string name;
    std::string type;
    std::string parent;
    std::string child;
    double origin_x {0.0};
    double origin_y {0.0};
    double origin_z {0.0};
    double rpy_x {0.0};
    double rpy_y {0.0};
    double rpy_z {0.0};
    double axis_x {0.0};
    double axis_y {1.0};
    double axis_z {0.0};
};

/**
 * @brief 展开后的 URDF 里我们关心的那点东西：link 的惯量、joint 的接线与原点
 */
struct Urdf
{
    std::vector<LinkInfo>  links;
    std::vector<JointInfo> joints;

    const LinkInfo& link(const std::string& name) const
    {
        for (const auto& l : links) {
            if (l.name == name) {
                return l;
            }
        }
        throw std::runtime_error("URDF 里没有 link：" + name);
    }

    const JointInfo& joint(const std::string& name) const
    {
        for (const auto& j : joints) {
            if (j.name == name) {
                return j;
            }
        }
        throw std::runtime_error("URDF 里没有 joint：" + name);
    }
};

void parse_floats(const char* text, std::vector<double>& out)
{
    out.assign(3, 0.0);
    if (text == nullptr) {
        return;
    }
    std::istringstream stream(text);
    for (std::size_t i = 0; i < out.size(); ++i) {
        if (!(stream >> out[i])) {
            break;
        }
    }
}

double attr_double(const tinyxml2::XMLElement* element, const char* name, double fallback)
{
    double value = fallback;
    element->QueryDoubleAttribute(name, &value);
    return value;
}

/**
 * @brief 解析展开后的 URDF 文本
 */
Urdf parse_urdf(const std::string& text)
{
    tinyxml2::XMLDocument document;
    if (document.Parse(text.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("URDF 不是合法 XML");
    }

    const tinyxml2::XMLElement* robot = document.FirstChildElement("robot");
    if (robot == nullptr) {
        throw std::runtime_error("URDF 没有 <robot> 根节点");
    }

    Urdf urdf;

    for (const tinyxml2::XMLElement* node = robot->FirstChildElement("link"); node != nullptr; node = node->NextSiblingElement("link")) 
    {
        LinkInfo link;
        link.name = node->Attribute("name");

        if (const tinyxml2::XMLElement* inertial = node->FirstChildElement("inertial")) {
            if (const tinyxml2::XMLElement* mass = inertial->FirstChildElement("mass")) {
                link.mass = attr_double(mass, "value", 0.0);
            }
            std::vector<double> xyz;
            std::vector<double> rpy;
            if (const tinyxml2::XMLElement* origin = inertial->FirstChildElement("origin")) {
                parse_floats(origin->Attribute("xyz"), xyz);
                parse_floats(origin->Attribute("rpy"), rpy);
            }
            link.com_x = xyz[0];
            link.com_z = xyz[2];
            link.inertial_roll = rpy[0];
            link.inertial_yaw  = rpy[2];

            if (const tinyxml2::XMLElement* inertia = inertial->FirstChildElement("inertia")) {
                link.ixx = attr_double(inertia, "ixx", 0.0);
                link.iyy = attr_double(inertia, "iyy", 0.0);
                link.izz = attr_double(inertia, "izz", 0.0);
            }
        }

        // 轮半径从几何里读：collision 优先，没有就看 visual
        const tinyxml2::XMLElement* shape = node->FirstChildElement("collision");
        if (shape == nullptr) {
            shape = node->FirstChildElement("visual");
        }
        if (shape != nullptr) {
            if (const tinyxml2::XMLElement* geometry = shape->FirstChildElement("geometry")) {
                if (const tinyxml2::XMLElement* cylinder = geometry->FirstChildElement("cylinder")) {
                    link.radius = attr_double(cylinder, "radius", 0.0);
                }
            }
        }

        urdf.links.push_back(link);
    }

    for (const tinyxml2::XMLElement* node = robot->FirstChildElement("joint"); node != nullptr; node = node->NextSiblingElement("joint")) 
    {
        JointInfo joint;
        joint.name = node->Attribute("name");
        joint.type = node->Attribute("type") ? node->Attribute("type") : "";

        if (const tinyxml2::XMLElement* parent = node->FirstChildElement("parent")) {
            joint.parent = parent->Attribute("link");
        }
        if (const tinyxml2::XMLElement* child = node->FirstChildElement("child")) {
            joint.child = child->Attribute("link");
        }
        if (const tinyxml2::XMLElement* origin = node->FirstChildElement("origin")) {
            std::vector<double> xyz;
            std::vector<double> rpy;
            parse_floats(origin->Attribute("xyz"), xyz);
            parse_floats(origin->Attribute("rpy"), rpy);
            joint.origin_x = xyz[0];
            joint.origin_y = xyz[1];
            joint.origin_z = xyz[2];
            joint.rpy_x = rpy[0];
            joint.rpy_y = rpy[1];
            joint.rpy_z = rpy[2];
        }
        if (const tinyxml2::XMLElement* axis = node->FirstChildElement("axis")) {
            std::vector<double> xyz;
            parse_floats(axis->Attribute("xyz"), xyz);
            joint.axis_x = xyz[0];
            joint.axis_y = xyz[1];
            joint.axis_z = xyz[2];
        }
        urdf.joints.push_back(joint);
    }

    return urdf;
}

std::string read_file(const std::string& path)
{
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("打不开文件：" + path);
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

/**
 * @brief .xacro 就先展开再解析；.urdf 直接读
 */
std::string load_urdf_text(const std::string& path)
{
    if (path.size() > 6 && path.compare(path.size() - 6, 6, ".xacro") == 0) {
        const std::string command = "xacro " + path;
        FILE* pipe = popen(command.c_str(), "r");
        if (pipe == nullptr) {
            throw std::runtime_error("起不了 xacro");
        }
        std::string text;
        std::array<char, 4096> buffer{};
        while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
            text += buffer.data();
        }
        const int status = pclose(pipe);
        if (status != 0 || text.empty()) {
            throw std::runtime_error("xacro 展开失败：" + path + "（记得先 source install/setup.bash）");
        }
        return text;
    }
    return read_file(path);
}

// ===========================================================================
// 平面共模模型
// ===========================================================================

/**
 * @brief 一个平面刚体：质量、绕 y 的惯量、COM 在本体坐标系里的 (x, z)
 */
struct PlanarBody
{
    std::string name;
    double mass {0.0};
    double inertia_yy {0.0};
    double com_x {0.0};
    double com_z {0.0};
};

/**
 * @brief 平面关节：父体 -> 子体，原点在父体坐标系里的 (x, z)，子体相对父体绕 +y 转
 */
struct PlanarJoint
{
    std::string name;
    double origin_x {0.0};
    double origin_z {0.0};
    double sign {1.0};
};

/**
 * @brief 平面共模机器人：机身 → 大腿 → 小腿 → 轮子，四个体三个关节
 *
 * @note 质量/惯量已经按"左右各一套"折过：腿的连杆 ×2，机身只有一份
 */
struct PlanarRobot
{
    std::array<PlanarBody, 4>  body {};
    std::array<PlanarJoint, 3> joint {};
    double wheel_radius {0.0};

    // 冻结腿姿之后的等效倒立摆参数，用来跟现有 4 状态模型对照
    double locked_mass {0.0};         // m：机身 + 两条腿（轮子除外）
    double locked_com_height {0.0};   // l：这个组合质心到轮轴的竖直距离
    double locked_pitch_inertia {0.0};// I：绕自身质心、绕 y
    double cart_mass {0.0};           // M：2*(轮子质量 + 自转惯量/r²)
    double total_mass {0.0};
};

/**
 * @brief 绕 +y 转 a 的二维旋转（x 向前、z 向上），与 URDF 的 axis=(0,1,0) 一致
 */
Eigen::Matrix2d rot_y(double a)
{
    Eigen::Matrix2d r;
    r << std::cos(a), std::sin(a),
        -std::sin(a), std::cos(a);
    return r;
}

/**
 * @brief 把固定关节挂上来的 link 并进父体：质量相加、质心按质量加权、惯量走平行轴
 *
 * @param parent 父体（合并前的）
 * @param child 固定关节的子 link（自己的惯量 origin 是相对自己 link 系的）
 * @param offset_x 固定关节原点在父体系里的 x
 * @param offset_z 同上，z
 * @param count 这种子 link 有几个：髋圆柱左右各一个都挂在机身上 → 2；
 *              膝圆柱每个大腿一个，左右那一份由后面的"腿连杆 ×2"统一乘 → 1
 * @note 调用处已经保证固定关节的 rpy 是 0。圆柱的惯量 origin 绕 x 转了 90°，
 *       所以它在 link 系里的 y 分量是**本体的 izz**（自转轴那一支），不是 iyy
 */
PlanarBody merge_fixed(const PlanarBody& parent, const LinkInfo& child, double offset_x, double offset_z, double count = 1.0)
{
    const bool rotated_90 = std::abs(child.inertial_roll - M_PI_2) < 1e-9;
    const double child_iy = rotated_90 ? child.izz : child.iyy;

    const double child_mass = child.mass * count;
    const double mass = parent.mass + child_mass;
    const double com_x = (parent.mass * parent.com_x + child_mass * (child.com_x + offset_x)) / mass;
    const double com_z = (parent.mass * parent.com_z + child_mass * (child.com_z + offset_z)) / mass;

    const double dp_x = parent.com_x - com_x;
    const double dp_z = parent.com_z - com_z;
    const double dc_x = child.com_x + offset_x - com_x;
    const double dc_z = child.com_z + offset_z - com_z;

    PlanarBody merged;
    merged.name       = parent.name;
    merged.mass       = mass;
    merged.com_x      = com_x;
    merged.com_z      = com_z;
    merged.inertia_yy = parent.inertia_yy + child_iy * count
                      + parent.mass * (dp_x * dp_x + dp_z * dp_z)
                      + child_mass * (dc_x * dc_x + dc_z * dc_z);
    return merged;
}

/**
 * @brief 从 URDF 拼出平面共模模型
 *
 * @param side 取哪条腿的几何（两条腿同构，取一边再 ×2；左右差异由 check_symmetry 单独查）
 * @throw std::runtime_error 链条、轴、惯量口径对不上时抛 —— 别拿半个模型往下算
 */
PlanarRobot build_planar_robot(const Urdf& urdf, const std::string& side)
{
    const JointInfo& hip   = urdf.joint(side + "_hip_joint");
    const JointInfo& knee  = urdf.joint(side + "_knee_joint");
    const JointInfo& wheel = urdf.joint(side + "_wheel_joint");

    for (const JointInfo* j : {&hip, &knee, &wheel}) {
        if (std::abs(j->axis_x) > 1e-9 || std::abs(j->axis_z) > 1e-9 || std::abs(j->axis_y) < 1e-9) {
            throw std::runtime_error("关节 " + j->name + " 的轴不是 y，平面模型不成立");
        }
        if (std::abs(j->rpy_x) > 1e-9 || std::abs(j->rpy_y) > 1e-9 || std::abs(j->rpy_z) > 1e-9) {
            throw std::runtime_error("关节 " + j->name + " 的原点带旋转，平面推导要重做");
        }
    }

    // 膝的原点：knee 关节的父是膝上的圆柱 link，圆柱由固定关节挂在大腿上，
    // 两个原点串起来才是"大腿系里的膝轴位置"
    const JointInfo& knee_cylinder = urdf.joint(side + "_knee_cylinder_joint");
    const JointInfo& hip_cylinder  = urdf.joint(side + "_hip_cylinder_joint");

    if (hip.parent != "base_link" || knee_cylinder.parent != hip.child ||
        knee.parent != knee_cylinder.child || wheel.parent != knee.child) {
        throw std::runtime_error("链条不是 base_link → 髋 → 大腿 →（膝圆柱）→ 小腿 → 轮子");
    }

    PlanarRobot robot;

    const LinkInfo& body_link   = urdf.link("base_link");
    const LinkInfo& thigh_link  = urdf.link(hip.child);
    const LinkInfo& calf_link   = urdf.link(knee.child);
    const LinkInfo& wheel_link  = urdf.link(wheel.child);
    const LinkInfo& hip_cyl     = urdf.link(hip_cylinder.child);
    const LinkInfo& knee_cyl    = urdf.link(knee_cylinder.child);

    for (const LinkInfo* l : {&body_link, &thigh_link, &calf_link, &wheel_link}) {
        if (std::abs(l->inertial_yaw) > 1e-9) {
            throw std::runtime_error("link " + l->name + " 的惯量 origin 带偏航，iyy 口径要重推");
        }
    }

    // 圆柱并进谁的体：按 URDF 里 fixed joint 的 parent 来 —— 髋圆柱挂在 base_link 上、
    // 膝圆柱挂在大腿上。之前按"跟谁一起转"猜，把两个都放错了位置，膝那份还多算了
    // 一个 span/2 的力臂（U0 差 0.001 N·m 就是它）
    robot.body[kBody]  = merge_fixed({"base_link", body_link.mass, body_link.iyy, body_link.com_x, body_link.com_z},
                                     hip_cyl, hip_cylinder.origin_x, hip_cylinder.origin_z, 2.0);
    robot.body[kThigh] = merge_fixed({hip.child, thigh_link.mass, thigh_link.iyy, thigh_link.com_x, thigh_link.com_z},
                                     knee_cyl, knee_cylinder.origin_x, knee_cylinder.origin_z);
    robot.body[kCalf]  = {knee.child, calf_link.mass, calf_link.iyy, calf_link.com_x, calf_link.com_z};
    robot.body[kWheel] = {wheel.child, wheel_link.mass, wheel_link.izz, wheel_link.com_x, wheel_link.com_z};

    // 腿的连杆左右各一套、共模同步 → ×2；机身平面上只有一份（横向宽度不进俯仰/前进）
    for (std::size_t i = kThigh; i <= kWheel; ++i) {
        robot.body[i].mass       *= 2.0;
        robot.body[i].inertia_yy *= 2.0;
    }

    robot.joint[0] = {"hip",   hip.origin_x,   hip.origin_z,   hip.axis_y   >= 0.0 ? 1.0 : -1.0};
    robot.joint[1] = {"knee",  knee_cylinder.origin_x + knee.origin_x,
                      knee_cylinder.origin_z + knee.origin_z,    knee.axis_y  >= 0.0 ? 1.0 : -1.0};
    robot.joint[2] = {"wheel", wheel.origin_x, wheel.origin_z,  wheel.axis_y >= 0.0 ? 1.0 : -1.0};

    robot.wheel_radius = wheel_link.radius;
    if (robot.wheel_radius <= 0.0) {
        throw std::runtime_error("轮子的 geometry 里没读到半径");
    }

    // 冻结腿姿的等效倒立摆参数（腿锁死时腿的连杆跟机身是一块刚体）：
    // 质量相加、质心加权、惯量用平行轴
    const double leg_mass = robot.body[kThigh].mass + robot.body[kCalf].mass;
    robot.locked_mass = robot.body[kBody].mass + leg_mass;

    // 腿锁死时，四个体的质心位置由运动学给出（在标准腿姿下），这里直接调一次运动学
    // 把质心压到"机身 + 两条腿"的合成质心上
    robot.total_mass = robot.body[kBody].mass + leg_mass + robot.body[kWheel].mass;
    robot.cart_mass  = robot.body[kWheel].mass + robot.body[kWheel].inertia_yy / (robot.wheel_radius * robot.wheel_radius);

    return robot;
}

// ===========================================================================
// 运动学
// ===========================================================================

/**
 * @brief 四个体的绝对角与世界系位置（轮心在 (s, r)，从轮心往上倒推）
 *
 * @param angles 可选：写出四个体的绝对角
 */
std::array<Eigen::Vector2d, 4> chain_positions(const PlanarRobot& robot, const GeneralVector& q, std::array<double, 4>* angles = nullptr)
{
    const double s  = q(kS);
    const double th = q(kTh);
    const double qh = q(kQh);
    const double qk = q(kQk);
    const double r  = robot.wheel_radius;

    const double psi_thigh = th + robot.joint[0].sign * qh;
    const double psi_calf  = psi_thigh + robot.joint[1].sign * qk;
    const double psi_wheel = psi_calf + robot.joint[2].sign * (s / r);   // 纯滚动约束：φ = s/r

    if (angles != nullptr) {
        (*angles)[kBody]  = th;
        (*angles)[kThigh] = psi_thigh;
        (*angles)[kCalf]  = psi_calf;
        (*angles)[kWheel] = psi_wheel;
    }

    const Eigen::Matrix2d r_body  = rot_y(th);
    const Eigen::Matrix2d r_thigh = rot_y(psi_thigh);
    const Eigen::Matrix2d r_calf  = rot_y(psi_calf);
    const Eigen::Matrix2d r_wheel = rot_y(psi_wheel);

    const Eigen::Vector2d wheel_origin(s, r);
    const Eigen::Vector2d calf_origin  = wheel_origin - r_calf  * Eigen::Vector2d(robot.joint[2].origin_x, robot.joint[2].origin_z);
    const Eigen::Vector2d thigh_origin = calf_origin  - r_thigh * Eigen::Vector2d(robot.joint[1].origin_x, robot.joint[1].origin_z);
    const Eigen::Vector2d body_origin  = thigh_origin - r_body  * Eigen::Vector2d(robot.joint[0].origin_x, robot.joint[0].origin_z);

    std::array<Eigen::Vector2d, 4> origins;
    origins[kBody]  = body_origin;
    origins[kThigh] = thigh_origin;
    origins[kCalf]  = calf_origin;
    origins[kWheel] = wheel_origin;
    return origins;
}

struct BodyKinematics
{
    Jacobian com_jacobian {Jacobian::Zero()};   // d(COM x,z)/d q
    RateRow  rate_row {RateRow::Zero()};        // d(绝对角)/d q
    double   com_x {0.0};
    double   com_z {0.0};
    double   angle {0.0};
};

/**
 * @brief q 处的全部运动学量：COM 位置、COM 雅可比（中心差分）、绝对角速度行（解析）
 */
std::array<BodyKinematics, 4> kinematics(const PlanarRobot& robot, const GeneralVector& q)
{
    std::array<double, 4> angles{};
    const auto origins = chain_positions(robot, q, &angles);

    std::array<BodyKinematics, 4> kin;
    for (std::size_t i = 0; i < 4; ++i) {
        const Eigen::Matrix2d rotation = rot_y(angles[i]);
        const Eigen::Vector2d com(robot.body[i].com_x, robot.body[i].com_z);
        const Eigen::Vector2d world = origins[i] + rotation * com;
        kin[i].com_x = world.x();
        kin[i].com_z = world.y();
        kin[i].angle = angles[i];
    }

    // COM 雅可比：对 q 中心差分（模型里全是三角函数，差分精度足够；步长在检查里扫）
    constexpr double step = 1e-6;
    for (std::size_t j = 0; j < kGeneralDim; ++j) {
        GeneralVector q_plus  = q;
        GeneralVector q_minus = q;
        q_plus(j)  += step;
        q_minus(j) -= step;
        const auto plus  = chain_positions(robot, q_plus);
        const auto minus = chain_positions(robot, q_minus);

        for (std::size_t i = 0; i < 4; ++i) {
            const Eigen::Matrix2d rp = rot_y(0.0);
            (void)rp;
            // 位置要连 COM 的旋转一起差：COM 世界位置 = 原点 + R(角)·c
            std::array<double, 4> ap{};
            std::array<double, 4> am{};
            const auto op = chain_positions(robot, q_plus, &ap);
            const auto om = chain_positions(robot, q_minus, &am);
            const Eigen::Vector2d c(robot.body[i].com_x, robot.body[i].com_z);
            const Eigen::Vector2d pp = op[i] + rot_y(ap[i]) * c;
            const Eigen::Vector2d pm = om[i] + rot_y(am[i]) * c;
            (void)plus;
            (void)minus;
            kin[i].com_jacobian(0, j) = (pp.x() - pm.x()) / (2.0 * step);
            kin[i].com_jacobian(1, j) = (pp.y() - pm.y()) / (2.0 * step);
        }
    }

    // 绝对角速度行：解析。ψ_大腿 = θ + q_h，ψ_小腿 = ψ_大腿 + q_k，ψ_轮子 = ψ_小腿 + s/r
    kin[kBody].rate_row.setZero();
    kin[kBody].rate_row(kTh) = 1.0;

    kin[kThigh].rate_row = kin[kBody].rate_row;
    kin[kThigh].rate_row(kQh) += robot.joint[0].sign;

    kin[kCalf].rate_row = kin[kThigh].rate_row;
    kin[kCalf].rate_row(kQk) += robot.joint[1].sign;

    // 轮子的**绝对**角被纯滚动钉死：ψ_轮 = s/r（不是"相对小腿等于 s/r"）。
    // 被电机驱动的是**相对**角 ψ_轮 − ψ_小腿，那一份在 S 矩阵里用
    kin[kWheel].rate_row.setZero();
    kin[kWheel].rate_row(kS) = robot.joint[2].sign / robot.wheel_radius;

    return kin;
}

// ===========================================================================
// 动力学
// ===========================================================================

/**
 * @brief M(q) = Σ [ m·JᵀJ + I·wᵀw ]
 */
GeneralMatrix mass_matrix(const PlanarRobot& robot, const GeneralVector& q)
{
    const auto kin = kinematics(robot, q);

    GeneralMatrix m = GeneralMatrix::Zero();
    for (std::size_t i = 0; i < 4; ++i) {
        m += robot.body[i].mass * kin[i].com_jacobian.transpose() * kin[i].com_jacobian;
        m += robot.body[i].inertia_yy * kin[i].rate_row.transpose() * kin[i].rate_row;
    }
    return m;
}

/**
 * @brief G(q) = ∂V/∂q，解析求导（不是差分）
 *
 * 每个体的高度都是若干 (a·sinψ + b·cosψ) 的和：
 *   z_i = r + Σ_{本体的位置链上那些关节} [d_x·sin ψ_父 − d_z·cos ψ_父] + [−c_x·sin ψ_i + c_z·cos ψ_i]
 * 其中 −(R(ψ)d)_z = d_x·sinψ − d_z·cosψ。对 q 求导 = Σ (a·cosψ − b·sinψ)·∂ψ/∂q，
 * ∂ψ/∂q 就是运动学里那几行 rate_row（也是解析的）。
 *
 * @note 必须解析：之前 G 用差分、∂G/∂q 又差分，两层差分的舍入误差被两次除法放大，
 *       步长扫描分不清"模型不稳"还是"差分不准"
 */
GeneralVector gravity_vector(const PlanarRobot& robot, const GeneralVector& q, double gravity)
{
    const auto kin = kinematics(robot, q);

    const std::array<RateRow, 4> rate{kin[kBody].rate_row, kin[kThigh].rate_row, kin[kCalf].rate_row, kin[kWheel].rate_row};
    const std::array<double, 4>  psi{kin[kBody].angle, kin[kThigh].angle, kin[kCalf].angle, kin[kWheel].angle};

    // a(i,v)/b(i,v)：第 i 个体的 z 里 sinψ_v / cosψ_v 的系数
    Eigen::Matrix<double, 4, 4> a = Eigen::Matrix<double, 4, 4>::Zero();
    Eigen::Matrix<double, 4, 4> b = Eigen::Matrix<double, 4, 4>::Zero();
    const auto add = [&](int body, int angle, double sa, double cb) {
        a(body, angle) += sa;
        b(body, angle) += cb;
    };

    // 关节 j 的原点项落在哪些体的链上：髋只落在机身上，膝落在机身+大腿，轮落在前三者；
    // 角用的是**父体**的角，而角的下标正好按 [机身, 大腿, 小腿, 轮子] 排，所以就是 j
    for (int joint = 0; joint < 3; ++joint) {
        for (int body = 0; body <= joint; ++body) {
            add(body, joint, robot.joint[joint].origin_x, -robot.joint[joint].origin_z);
        }
    }
    // COM 项：(R(ψ)c)_z = −c_x·sinψ + c_z·cosψ
    for (int i = 0; i < 4; ++i) {
        add(i, i, -robot.body[i].com_x, robot.body[i].com_z);
    }

    GeneralVector g = GeneralVector::Zero();
    for (int i = 0; i < 4; ++i) 
    {
        for (int v = 0; v < 4; ++v) {
            const double derivative = a(i, v) * std::cos(psi[v]) - b(i, v) * std::sin(psi[v]);
            g += robot.body[i].mass * gravity * derivative * rate[v].transpose();
        }
    }
    return g;
}

/**
 * @brief S：每个输入作用在对应关节的**相对转角**上，行 = ∂(相对角)/∂q，共模两侧各一个 → ×2
 *
 * @note 轮子那一路驱动的是"轮相对小腿"的转角 = ψ_轮 − ψ_小腿 = s/r − ψ_小腿，
 *       所以它同时对 θ、髋、膝有反作用 —— 轮力矩对车身俯仰的控制权全在这一列上，
 *       漏掉后面几项的话 B 里轮子那一路就只剩水平推力，跟倒立摆模型对不上
 */
GeneralInput input_matrix(const PlanarRobot& robot)
{
    // ∂ψ_小腿/∂q = [0, 1, sign_hip, sign_knee]
    const Eigen::Matrix<double, kGeneralDim, 1> psi_calf_q = (Eigen::Matrix<double, kGeneralDim, 1>()
        << 0.0, 1.0, robot.joint[0].sign, robot.joint[1].sign).finished();

    GeneralInput s = GeneralInput::Zero();
    s.col(kTauWheel) = -2.0 * psi_calf_q;
    s(kS,  kTauWheel) += 2.0 * robot.joint[2].sign / robot.wheel_radius;
    s(kQh, kTauHip)    = 2.0;
    s(kQk, kTauKnee)   = 2.0;
    return s;
}

// ===========================================================================
// 线性化与离散
// ===========================================================================

/**
 * @brief 静止工作点上的 A/B，输出按 [q0, q0_dot, q1, q1_dot, ...] 交错排列
 *
 * @note 工作点静止（q̇ = 0），C(q,0) = 0 且 ∂(Cq̇)/∂q|₀ = 0，所以 A/B 里科氏项不出现
 * @param step ∂G/∂q 的中心差分步长
 */
std::pair<StateMatrix, InputMatrix> linearize(const PlanarRobot& robot, const GeneralVector& q0, double gravity, double step)
{
    const GeneralMatrix m_inv = mass_matrix(robot, q0).inverse();
    const GeneralInput  s     = input_matrix(robot);

    constexpr std::size_t kJointCount = 4;
    GeneralMatrix dg = GeneralMatrix::Zero();

    for (std::size_t j = 0; j < kJointCount; ++j) {
        GeneralVector q_plus  = q0;
        GeneralVector q_minus = q0;
        q_plus(j)  += step;
        q_minus(j) -= step;
        dg.col(j) = (gravity_vector(robot, q_plus, gravity) - gravity_vector(robot, q_minus, gravity)) / (2.0 * step);
    }

    StateMatrix a = StateMatrix::Zero();
    InputMatrix b = InputMatrix::Zero();

    const GeneralMatrix acceleration_a = -m_inv * dg;
    for (std::size_t i = 0; i < kJointCount; ++i) {
        const std::size_t position_row = 2 * i;
        const std::size_t velocity_row = position_row + 1;
        a(position_row, velocity_row) = 1.0;                           // dq/dt = q̇
        for (std::size_t j = 0; j < kJointCount; ++j) {
            a(velocity_row, 2 * j) = acceleration_a(i, j);             // q̈ = −M⁻¹ ∂G/∂q · q
        }
    }

    const Eigen::Matrix<double, kJointCount, kInputDim> m_inv_s = m_inv * s;
    for (std::size_t i = 0; i < kJointCount; ++i) {
        for (std::size_t j = 0; j < kInputDim; ++j) {
            b(2 * i + 1, j) = m_inv_s(i, j);
        }
    }

    return {a, b};
}

/**
 * @brief 矩阵指数：缩放-平方 + 泰勒（|A| 不大，12 项足够）
 */
Eigen::MatrixXd matrix_exp(const Eigen::MatrixXd& a)
{
    const int n = static_cast<int>(a.rows());
    double norm = a.cwiseAbs().rowwise().sum().maxCoeff();

    int squarings = 0;
    while (norm > 0.5) {
        norm *= 0.5;
        ++squarings;
    }

    const Eigen::MatrixXd scaled = a / std::pow(2.0, squarings);
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd term = result;
    for (int k = 1; k <= 12; ++k) {
        term = term * scaled / static_cast<double>(k);
        result += term;
    }
    for (int k = 0; k < squarings; ++k) {
        result = result * result;
    }
    return result;
}

/**
 * @brief 零阶保持离散：Ad = exp(A·dt)，Bd = ∫exp(A·t)B dt（用增广矩阵一次算出来）
 *
 * @note 尺寸从入参取：这套离散化矢状面（8×3）和 Roll（4×2）都要用
 */
template <typename AType, typename BType>
std::pair<AType, BType> discretize(const AType& a, const BType& b, double dt)
{
    const int n = static_cast<int>(a.rows());
    const int m = static_cast<int>(b.cols());

    Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(n + m, n + m);
    augmented.topLeftCorner(n, n) = a * dt;
    augmented.topRightCorner(n, m) = b * dt;

    const Eigen::MatrixXd result = matrix_exp(augmented);

    AType ad = result.topLeftCorner(n, n);
    BType bd = result.topRightCorner(n, m);
    return {ad, bd};
}

// ===========================================================================
// Roll（差模）子系统：前视面平面模型
//
// 为什么只有两个广义坐标 [q_h−, q_k−]：
//   两个轮子都贴地时，左右腿的高度差唯一决定机身 Roll —— 轮距、髋距都是刚性的，
//   把轮心钉在地面上解一遍几何就能得到 φ = atan2(h_L − h_R, 2·wheel_y)（推到下面
//   frontal_kinematics 里）。也就是说 roll 不是独立自由度，它就是腿差模的另一种看法。
//   硬按文档 §3.3 的 6 状态写（roll、roll_rate 也当状态），A 里会多一对零特征值的
//   约束模态，可控性直接掉秩。所以这里用 4 状态：[q_h−, q̇_h−, q_k−, q̇_k−]。
//
// 近似（都在 1e-05 量级，比主导项小三个数量级，写在注释里不藏着）：
//   · 各连杆的自转惯量取 URDF 的 ixx 原值（它的惯量 origin 绕 y 转过，x 轴名义上会混一点
//     z 分量；对细杆是 1e-06 级）
//   · 轮子当作随机身一起滚（它的 camber 实际被地面约束住），贡献 1e-04 级
// ===========================================================================

constexpr std::size_t kRollGeneralDim = 2;   // 广义坐标 [q_h−, q_k−]
constexpr std::size_t kRollStateDim   = 4;   // 状态 [q_h−, q̇_h−, q_k−, q̇_k−]
constexpr std::size_t kRollInputDim   = 2;   // 输入 [τ_h−, τ_k−]

const std::array<const char*, kRollStateDim> kRollStateNames{"q_h_diff", "q_h_diff_rate", "q_k_diff", "q_k_diff_rate"};
const std::array<const char*, kRollInputDim> kRollInputNames{"tau_h_diff", "tau_k_diff"};

using RollGeneralVector = Eigen::Matrix<double, kRollGeneralDim, 1>;
using RollStateMatrix   = Eigen::Matrix<double, kRollStateDim, kRollStateDim>;
using RollInputMatrix   = Eigen::Matrix<double, kRollStateDim, kRollInputDim>;
using RollControlMatrix = Eigen::Matrix<double, kRollInputDim, kRollInputDim>;
using RollGeneralMatrix = Eigen::Matrix<double, kRollGeneralDim, kRollGeneralDim>;
using RollGeneralInput  = Eigen::Matrix<double, kRollGeneralDim, kRollInputDim>;
using RollJacobian      = Eigen::Matrix<double, 2, kRollGeneralDim>;
using RollRateRow       = Eigen::Matrix<double, 1, kRollGeneralDim>;

/**
 * @brief Roll 模型的一个刚体：质量、绕 x 的自身惯量、在**机身系**里的质心 (x, y, z)
 *
 * @note 腿的连杆/轮子：y 是它自己那条链的横向位置（大腿小腿 ±hip_y、轮 ±wheel_y），
 *       (x, z) 用那条腿自己的髋膝角从矢状链上算，z 以机身原点（轮轴高度）为零点
 */
struct RollBody
{
    std::string name;
    double      mass {0.0};
    double      inertia_x {0.0};
    double      com_x {0.0};
    double      com_y {0.0};
    double      com_z {0.0};
};

/**
 * @brief 前视面模型：机身 + 左右各三条腿连杆 + 两个轮子
 */
struct FrontalRobot
{
    static constexpr std::size_t kBody = 0;
    static constexpr std::size_t kThighL = 1;
    static constexpr std::size_t kThighR = 2;
    static constexpr std::size_t kCalfL = 3;
    static constexpr std::size_t kCalfR = 4;
    static constexpr std::size_t kWheelL = 5;
    static constexpr std::size_t kWheelR = 6;

    std::array<RollBody, 7> body {};
    double hip_y {0.0};          // 髋轴的横向位置
    double wheel_y {0.0};        // 轮心的横向位置
    double hip_height {0.0};     // 机身系里髋轴的高度
    double wheel_radius {0.0};
    double common_hip {0.0};     // 当前调度工作点的髋共模角
    double common_knee {0.0};    // 当前调度工作点的膝共模角
    PlanarRobot sagittal;        // 借它的矢状链算每条腿的 h 和连杆质心
};

/**
 * @brief 从 URDF + 已有的矢状模型拼出前视面模型
 *
 * @note 腿的连杆质量 = 矢状模型里那个"左右合一"的体 ÷ 2（矢状体是共模求和过的），
 *       质心和惯量不用除 —— 左右同构，取一半质量、同一个质心就还原成单侧
 */
FrontalRobot build_frontal_robot(const Urdf& urdf, const std::string& side, const PlanarRobot& sagittal)
{
    const JointInfo& hip   = urdf.joint(side + "_hip_joint");
    const JointInfo& wheel = urdf.joint(side + "_wheel_joint");

    FrontalRobot frontal;
    frontal.hip_y        = std::abs(hip.origin_y);
    frontal.wheel_y      = std::abs(wheel.origin_y);
    frontal.hip_height   = hip.origin_z;
    frontal.wheel_radius = sagittal.wheel_radius;
    frontal.sagittal     = sagittal;

    // 机身：质量/质心用矢状模型那份（它把两个髋圆柱按 fixed joint 的 parent 并进了机身）。
    // 绕 x 的惯量要另算：base_link 的 ixx + 两个髋圆柱（自身 ixx + 到合成质心的平行轴）
    const LinkInfo& body_link = urdf.link("base_link");
    const LinkInfo& cylinder  = urdf.link(urdf.joint(side + "_hip_cylinder_joint").child);

    const double com_z  = sagittal.body[kBody].com_z;
    const double dz_cyl = frontal.hip_height - com_z;                 // 圆柱质心在髋轴上
    const double body_inertia_x = body_link.ixx
        + 2.0 * (cylinder.ixx + cylinder.mass * (frontal.hip_y * frontal.hip_y + dz_cyl * dz_cyl));

    frontal.body[FrontalRobot::kBody] = {"base_link", sagittal.body[kBody].mass, body_inertia_x,
                                         0.0, 0.0, com_z};

    // 腿的六条：质量取矢状体的一半（矢状体是左右共模求和过的），质心位置随那条腿自己的角变，
    // 真正的位置在 frontal_kinematics 里按角算，这里只填质量/横向位置/自身惯量
    const struct { std::size_t index; int type; double y; } legs[] = {
        {FrontalRobot::kThighL, 0, +frontal.hip_y},
        {FrontalRobot::kThighR, 0, -frontal.hip_y},
        {FrontalRobot::kCalfL,  1, +frontal.hip_y},
        {FrontalRobot::kCalfR,  1, -frontal.hip_y},
        {FrontalRobot::kWheelL, 2, +frontal.wheel_y},
        {FrontalRobot::kWheelR, 2, -frontal.wheel_y},
    };

    for (const auto& leg : legs) {
        const int type = leg.type;   // 0 大腿、1 小腿、2 轮子（矢状体下标）
        const PlanarBody& source = sagittal.body[type == 0 ? kThigh : (type == 1 ? kCalf : kWheel)];
        const LinkInfo& link = (type == 0)   ? urdf.link(hip.child)
                             : (type == 1)   ? urdf.link(urdf.joint(side + "_knee_joint").child)
                                             : urdf.link(wheel.child);

        RollBody body;
        body.name      = link.name;
        body.mass      = source.mass * 0.5;
        body.com_x     = source.com_x;
        body.com_y     = leg.y;
        body.com_z     = source.com_z;
        body.inertia_x = link.ixx;
        frontal.body[leg.index] = body;
    }

    // 大腿的 x 惯量里还含它自己的膝圆柱（质量/质心矢状模型已经并过了，这里只补自身惯量）
    {
        const LinkInfo& knee_cyl = urdf.link(urdf.joint(side + "_knee_cylinder_joint").child);
        for (std::size_t index : {FrontalRobot::kThighL, FrontalRobot::kThighR}) {
            frontal.body[index].inertia_x += knee_cyl.ixx;
        }
    }

    return frontal;
}

/**
 * @brief 一条腿的矢状链：给定这条腿的髋膝角，返回髋轴高度 h 与各连杆在机身系里的 (x, z)
 *
 * @note 复用矢状模型：轮心放在 (0, r)，往上推。返回的 z 减去 r 就是机身系（原点在轮轴）的量
 */
struct SideChain
{
    double hip_z {0.0};           // 髋轴离地高度 = r + h
    double thigh_x {0.0};
    double thigh_z {0.0};
    double calf_x {0.0};
    double calf_z {0.0};
    double wheel_x {0.0};
    double wheel_z {0.0};
};

/**
 * @brief 一条腿的高度量对 [q_h, q_k] 的解析导数
 */
struct SideChainGradient
{
    Eigen::Vector2d hip_z   {Eigen::Vector2d::Zero()};
    Eigen::Vector2d thigh_z {Eigen::Vector2d::Zero()};
    Eigen::Vector2d calf_z  {Eigen::Vector2d::Zero()};
};

/**
 * @brief R_y(angle) * point 的 z 分量及对 angle 的导数
 */
std::pair<double, double> rotated_z_with_derivative(double angle, const Eigen::Vector2d& point)
{
    const double s = std::sin(angle);
    const double c = std::cos(angle);
    return {-s * point.x() + c * point.y(), -c * point.x() - s * point.y()};
}

SideChainGradient side_chain_gradient(const PlanarRobot& robot, double hip_angle, double knee_angle)
{
    const double thigh_angle = robot.joint[0].sign * hip_angle;
    const double calf_angle  = thigh_angle + robot.joint[1].sign * knee_angle;
    const Eigen::Vector2d knee_offset(robot.joint[1].origin_x, robot.joint[1].origin_z);
    const Eigen::Vector2d wheel_offset(robot.joint[2].origin_x, robot.joint[2].origin_z);
    const Eigen::Vector2d thigh_com(robot.body[kThigh].com_x, robot.body[kThigh].com_z);
    const Eigen::Vector2d calf_com(robot.body[kCalf].com_x, robot.body[kCalf].com_z);

    const auto [wheel_z, wheel_dz] = rotated_z_with_derivative(calf_angle, wheel_offset);
    const auto [knee_z, knee_dz]   = rotated_z_with_derivative(thigh_angle, knee_offset);
    const auto [thigh_com_z, thigh_com_dz] = rotated_z_with_derivative(thigh_angle, thigh_com);
    const auto [calf_com_z, calf_com_dz]   = rotated_z_with_derivative(calf_angle, calf_com);
    (void)wheel_z;
    (void)knee_z;
    (void)thigh_com_z;
    (void)calf_com_z;

    const Eigen::Vector2d thigh_angle_gradient(robot.joint[0].sign, 0.0);
    const Eigen::Vector2d calf_angle_gradient(robot.joint[0].sign, robot.joint[1].sign);
    const Eigen::Vector2d calf_origin_gradient = -wheel_dz * calf_angle_gradient;
    const Eigen::Vector2d thigh_origin_gradient = calf_origin_gradient - knee_dz * thigh_angle_gradient;

    SideChainGradient gradient;
    gradient.hip_z   = thigh_origin_gradient;
    gradient.thigh_z = thigh_origin_gradient + thigh_com_dz * thigh_angle_gradient;
    gradient.calf_z  = calf_origin_gradient + calf_com_dz * calf_angle_gradient;
    return gradient;
}

SideChain side_chain(const PlanarRobot& robot, double hip_angle, double knee_angle)
{
    GeneralVector q = GeneralVector::Zero();
    q(kQh) = hip_angle;
    q(kQk) = knee_angle;

    const auto positions = chain_positions(robot, q);   // [机身, 大腿, 小腿, 轮子] 的原点

    // 链上各点的原点：用 origin 加自身 COM 的旋转
    std::array<double, 4> angles{};
    chain_positions(robot, q, &angles);

    const auto com_of = [&](int index) {
        const Eigen::Vector2d c(robot.body[index].com_x, robot.body[index].com_z);
        const Eigen::Vector2d world = positions[index] + rot_y(angles[index]) * c;
        return world;
    };

    const Eigen::Vector2d thigh = com_of(kThigh);
    const Eigen::Vector2d calf  = com_of(kCalf);
    const Eigen::Vector2d wheel = com_of(kWheel);

    SideChain chain;
    chain.hip_z   = positions[kThigh].y();          // 大腿 link 的原点就是髋轴（link 原点在近端关节上）
    chain.thigh_x = thigh.x();
    chain.thigh_z = thigh.y();
    chain.calf_x  = calf.x();
    chain.calf_z  = calf.y();
    chain.wheel_x = wheel.x();
    chain.wheel_z = wheel.y();
    return chain;
}

/**
 * @brief 前视面运动学：给定 [q_h−, q_k−]，算 Roll 和七个体的世界坐标 (y, z)
 *
 * 共模腿姿固定在当前调度工作点，左右分别是 q+ ± q−：
 *   h_L, h_R    两条腿的髋轴高度（由各自的矢状链给出）
 *   φ           机身 Roll：轮心钉在地面上解几何 ——
 *               髋在机身系 (sign·hip_y, hip_height)，轮心在机身系的横向为 sign·wheel_y。
 *               两轮同时贴平地要求它们的世界系 z 相同，因此
 *                 tanφ = (h_L − h_R)/(2·wheel_y)
 *               Roll 后轮心的世界系横向间距不再是常数 2·wheel_y，而是
 *                 D = hypot(2·wheel_y, h_L − h_R)。
 *               把 D 也算进去，左右两条腿才会给出完全相同的机身原点。
 *   然后 O 由左边那条定出来，右边那条用来验（残差打进检查）
 */
struct FrontalKinematics
{
    double roll {0.0};
    std::array<double, 7> y {};
    std::array<double, 7> z {};
    double constraint_residual {0.0};    // 右边那条腿的几何残差，应该 ~0
};

FrontalKinematics frontal_kinematics(const FrontalRobot& frontal, const RollGeneralVector& r)
{
    const double q_h_minus = r(0);
    const double q_k_minus = r(1);

    const SideChain left  = side_chain(frontal.sagittal,
                                       frontal.common_hip + q_h_minus,
                                       frontal.common_knee + q_k_minus);
    const SideChain right = side_chain(frontal.sagittal,
                                       frontal.common_hip - q_h_minus,
                                       frontal.common_knee - q_k_minus);

    const double h_left  = left.hip_z - frontal.wheel_radius;
    const double h_right = right.hip_z - frontal.wheel_radius;

    const double track = 2.0 * frontal.wheel_y;
    const double height_difference = h_left - h_right;
    const double roll = std::atan2(height_difference, track);
    const double world_track = std::hypot(track, height_difference);

    // 机身原点：由左侧推，再验右侧
    const double c = std::cos(roll);
    const double s = std::sin(roll);
    const auto rotate = [&](double y, double z) { return std::pair<double, double>(y * c - z * s, y * s + z * c); };

    // 机身原点：O = (D/2, r) + R(φ)·(−wheel_y, h_L − hip_height)，由左侧定出来
    const auto left_rotated = rotate(-frontal.wheel_y, h_left - frontal.hip_height);
    const double oy = 0.5 * world_track + left_rotated.first;
    const double oz = frontal.wheel_radius + left_rotated.second;

    // 右侧一致性：O 也应该等于 (−D/2, r) + R(φ)·(wheel_y, h_R − hip_height)
    const auto right_rotated = rotate(frontal.wheel_y, h_right - frontal.hip_height);
    const double right_y = -0.5 * world_track + right_rotated.first;
    const double right_z = frontal.wheel_radius + right_rotated.second;

    FrontalKinematics kin;
    kin.roll = roll;
    kin.constraint_residual = std::hypot(oy - right_y, oz - right_z);

    const auto place = [&](std::size_t index, double com_y, double com_z) {
        const auto rotated = rotate(com_y, com_z);
        kin.y[index] = oy + rotated.first;
        kin.z[index] = oz + rotated.second;
    };

    // 机身：机身系里质心在 (0, com_z)，横向为 0
    place(FrontalRobot::kBody, 0.0, frontal.body[FrontalRobot::kBody].com_z);

    // 腿 COM 先写成髋轴系里的高度：hip_height + (COM_z - hip_z)。
    // 不能直接用 COM_z-r；后者等于让每条腿各自带一个机身原点。
    const auto leg_offset = [&](std::size_t index, const RollBody& body,
                                double com_z_above_ground, double hip_z_above_ground) {
        place(index, body.com_y, frontal.hip_height + com_z_above_ground - hip_z_above_ground);
    };

    leg_offset(FrontalRobot::kThighL, frontal.body[FrontalRobot::kThighL], left.thigh_z, left.hip_z);
    leg_offset(FrontalRobot::kThighR, frontal.body[FrontalRobot::kThighR], right.thigh_z, right.hip_z);
    leg_offset(FrontalRobot::kCalfL,  frontal.body[FrontalRobot::kCalfL],  left.calf_z, left.hip_z);
    leg_offset(FrontalRobot::kCalfR,  frontal.body[FrontalRobot::kCalfR],  right.calf_z, right.hip_z);

    // 轮心是链的端点；在机身系的 z 由本侧腿高决定。经上面的精确 φ 变换后两边都应回到 z=r。
    place(FrontalRobot::kWheelL, +frontal.wheel_y, frontal.hip_height - h_left);
    place(FrontalRobot::kWheelR, -frontal.wheel_y, frontal.hip_height - h_right);

    return kin;
}

/**
 * @brief Roll 的 M/G：M = Σ(m·JᵀJ + I_x·wᵀw)，G = Σ m·g·∂z/∂q，w = ∂φ/∂q
 *
 * @note M 的位置雅可比用中心差分；G 在 frontal_gravity_vector() 里按链式法则解析计算，
 *       避免线性化时出现"差分套差分"。
 */
RollGeneralMatrix frontal_mass_matrix(const FrontalRobot& frontal, const RollGeneralVector& r)
{
    constexpr double step = 1e-7;   // 只用于 M 里的质心位置/角度雅可比

    // 按定义拼：每个体的 J_i = ∂(y,z)/∂q 是 2×2，w_i = ∂φ/∂q 是 1×2
    RollGeneralMatrix m = RollGeneralMatrix::Zero();
    for (std::size_t i = 0; i < 7; ++i) {
        RollJacobian jacobian = RollJacobian::Zero();
        RollRateRow rate = RollRateRow::Zero();
        for (std::size_t j = 0; j < kRollGeneralDim; ++j) {
            RollGeneralVector plus  = r;
            RollGeneralVector minus = r;
            plus(j)  += step;
            minus(j) -= step;
            const auto kp = frontal_kinematics(frontal, plus);
            const auto km = frontal_kinematics(frontal, minus);
            jacobian(0, j) = (kp.y[i] - km.y[i]) / (2.0 * step);
            jacobian(1, j) = (kp.z[i] - km.z[i]) / (2.0 * step);
            rate(0, j)     = (kp.roll - km.roll) / (2.0 * step);
        }
        m += frontal.body[i].mass * jacobian.transpose() * jacobian;
        m += frontal.body[i].inertia_x * rate.transpose() * rate;
    }
    return m;
}

RollGeneralVector frontal_gravity_vector(const FrontalRobot& frontal, const RollGeneralVector& r, double gravity)
{
    const double q_h_minus = r(0);
    const double q_k_minus = r(1);
    const SideChain left  = side_chain(frontal.sagittal,
                                       frontal.common_hip + q_h_minus,
                                       frontal.common_knee + q_k_minus);
    const SideChain right = side_chain(frontal.sagittal,
                                       frontal.common_hip - q_h_minus,
                                       frontal.common_knee - q_k_minus);
    const SideChainGradient left_side_gradient =
        side_chain_gradient(frontal.sagittal,
                            frontal.common_hip + q_h_minus,
                            frontal.common_knee + q_k_minus);
    const SideChainGradient right_side_gradient =
        side_chain_gradient(frontal.sagittal,
                            frontal.common_hip - q_h_minus,
                            frontal.common_knee - q_k_minus);

    // 右腿的实际角是差模坐标的负号，链式法则要再乘 -1。
    const Eigen::Vector2d h_left_gradient = left_side_gradient.hip_z;
    const Eigen::Vector2d h_right_gradient = -right_side_gradient.hip_z;
    const double h_left  = left.hip_z - frontal.wheel_radius;
    const double h_right = right.hip_z - frontal.wheel_radius;
    const double height_difference = h_left - h_right;
    const Eigen::Vector2d height_difference_gradient = h_left_gradient - h_right_gradient;

    const double track = 2.0 * frontal.wheel_y;
    const double roll = std::atan2(height_difference, track);
    const double c = std::cos(roll);
    const double s = std::sin(roll);
    const Eigen::Vector2d roll_gradient =
        (track / (track * track + height_difference * height_difference)) * height_difference_gradient;

    // O_z = r - wheel_y*sin(phi) + (h_L-hip_height)*cos(phi)
    const Eigen::Vector2d origin_z_gradient = h_left_gradient * c
        + (-frontal.wheel_y * c - (h_left - frontal.hip_height) * s) * roll_gradient;

    std::array<double, 7> local_z{};
    std::array<Eigen::Vector2d, 7> local_z_gradient{};
    local_z[FrontalRobot::kBody] = frontal.body[FrontalRobot::kBody].com_z;

    local_z[FrontalRobot::kThighL] = frontal.hip_height + left.thigh_z - left.hip_z;
    local_z_gradient[FrontalRobot::kThighL] = left_side_gradient.thigh_z - left_side_gradient.hip_z;
    local_z[FrontalRobot::kThighR] = frontal.hip_height + right.thigh_z - right.hip_z;
    local_z_gradient[FrontalRobot::kThighR] = -(right_side_gradient.thigh_z - right_side_gradient.hip_z);

    local_z[FrontalRobot::kCalfL] = frontal.hip_height + left.calf_z - left.hip_z;
    local_z_gradient[FrontalRobot::kCalfL] = left_side_gradient.calf_z - left_side_gradient.hip_z;
    local_z[FrontalRobot::kCalfR] = frontal.hip_height + right.calf_z - right.hip_z;
    local_z_gradient[FrontalRobot::kCalfR] = -(right_side_gradient.calf_z - right_side_gradient.hip_z);

    local_z[FrontalRobot::kWheelL] = frontal.hip_height - h_left;
    local_z_gradient[FrontalRobot::kWheelL] = -h_left_gradient;
    local_z[FrontalRobot::kWheelR] = frontal.hip_height - h_right;
    local_z_gradient[FrontalRobot::kWheelR] = -h_right_gradient;

    RollGeneralVector g = RollGeneralVector::Zero();
    for (std::size_t i = 0; i < frontal.body.size(); ++i) {
        // z_i = O_z + y_i*sin(phi) + z_i_body*cos(phi)
        const Eigen::Vector2d world_z_gradient = origin_z_gradient
            + frontal.body[i].com_y * c * roll_gradient
            + local_z_gradient[i] * c
            - local_z[i] * s * roll_gradient;
        g += frontal.body[i].mass * gravity * world_z_gradient;
    }
    return g;
}

double frontal_potential(const FrontalRobot& frontal, const RollGeneralVector& r, double gravity)
{
    const FrontalKinematics kin = frontal_kinematics(frontal, r);
    double potential = 0.0;
    for (std::size_t i = 0; i < frontal.body.size(); ++i) {
        potential += frontal.body[i].mass * gravity * kin.z[i];
    }
    return potential;
}

/**
 * @brief Roll 的 S：差模输入做功在 q_h− / q_k− 上，左右各反号 → 系数是 2
 */
RollGeneralInput frontal_input_matrix()
{
    RollGeneralInput s = RollGeneralInput::Zero();
    s(0, 0) = 2.0;   // τ_h− 作用在 q_h− 上（左边 +、右边 −，两边同向贡献）
    s(1, 1) = 2.0;   // τ_k−
    return s;
}

/**
 * @brief Roll 子系统的 A/B：跟矢状面同一套写法（工作点静止，科氏项不出现）
 */
std::pair<RollStateMatrix, RollInputMatrix> frontal_linearize(const FrontalRobot& frontal,
                                                             const RollGeneralVector& r0,
                                                             double gravity, double step)
{
    const RollGeneralMatrix m_inv = frontal_mass_matrix(frontal, r0).inverse();
    const RollGeneralInput  s     = frontal_input_matrix();

    RollGeneralMatrix dg = RollGeneralMatrix::Zero();
    for (std::size_t j = 0; j < kRollGeneralDim; ++j) {
        RollGeneralVector plus  = r0;
        RollGeneralVector minus = r0;
        plus(j)  += step;
        minus(j) -= step;
        dg.col(j) = (frontal_gravity_vector(frontal, plus, gravity)
                   - frontal_gravity_vector(frontal, minus, gravity)) / (2.0 * step);
    }

    RollStateMatrix a = RollStateMatrix::Zero();
    RollInputMatrix b = RollInputMatrix::Zero();
    const RollGeneralMatrix acc = -m_inv * dg;
    const RollGeneralMatrix m_inv_s = m_inv * s;

    for (std::size_t i = 0; i < kRollGeneralDim; ++i) {
        const std::size_t position_row = 2 * i;
        const std::size_t velocity_row = position_row + 1;
        a(position_row, velocity_row) = 1.0;
        for (std::size_t j = 0; j < kRollGeneralDim; ++j) {
            a(velocity_row, 2 * j) = acc(i, j);
        }
        for (std::size_t j = 0; j < kRollInputDim; ++j) {
            b(velocity_row, j) = m_inv_s(i, j);
        }
    }
    return {a, b};
}

// ===========================================================================
// 小工具：检查、读参数、写 yaml
// ===========================================================================

double parse_list_entry(const std::string& text, const std::string& key, std::size_t index, double fallback)
{
    const std::size_t key_at = text.find(key + ":");
    if (key_at == std::string::npos) {
        return fallback;
    }
    const std::size_t open  = text.find('[', key_at);
    const std::size_t close = text.find(']', open);
    if (open == std::string::npos || close == std::string::npos) {
        return fallback;
    }

    std::istringstream stream(text.substr(open + 1, close - open - 1));
    std::string token;
    std::size_t current = 0;
    while (std::getline(stream, token, ',')) {
        if (current == index) {
            try {
                return std::stod(token);
            } catch (const std::exception&) {
                return fallback;
            }
        }
        ++current;
    }
    return fallback;
}

bool all_finite(const Eigen::MatrixXd& m)
{
    return m.allFinite();
}

int rank_of(const Eigen::MatrixXd& m, double tolerance)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m);
    const auto singular = svd.singularValues();
    int rank = 0;
    for (int i = 0; i < singular.size(); ++i) {
        if (singular(i) > tolerance) {
            ++rank;
        }
    }
    return rank;
}

std::string matrix_to_yaml(const Eigen::MatrixXd& m)
{
    std::ostringstream os;
    os << std::setprecision(12);
    os << "[";
    for (int i = 0; i < m.rows(); ++i) {
        os << (i == 0 ? "" : ",\n      ") << "[";
        for (int j = 0; j < m.cols(); ++j) {
            os << (j == 0 ? "" : ", ") << m(i, j);
        }
        os << "]";
    }
    os << "]";
    return os.str();
}

std::string vector_to_yaml(const Eigen::VectorXd& v)
{
    std::ostringstream os;
    os << std::setprecision(12);
    os << "[";
    for (int i = 0; i < v.size(); ++i) {
        os << (i == 0 ? "" : ", ") << v(i);
    }
    os << "]";
    return os.str();
}

std::uint64_t fnv1a(const std::string& text)
{
    std::uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char c : text) {
        hash ^= c;
        hash *= 1099511628211ULL;
    }
    return hash;
}

/**
 * @brief 去掉 xacro 生成头里的源文件路径，让 source/install 两份展开 URDF 得到同一哈希
 */
std::string model_hash_text(const std::string& xml)
{
    std::string uncommented;
    uncommented.reserve(xml.size());

    std::size_t cursor = 0;
    while (cursor < xml.size()) {
        const std::size_t comment = xml.find("<!--", cursor);
        if (comment == std::string::npos) {
            uncommented.append(xml, cursor, std::string::npos);
            break;
        }
        uncommented.append(xml, cursor, comment - cursor);
        const std::size_t end = xml.find("-->", comment + 4);
        if (end == std::string::npos) {
            throw std::runtime_error("URDF 里的 XML 注释没有闭合");
        }
        cursor = end + 3;
    }
    std::string normalized;
    normalized.reserve(uncommented.size());
    char quote = '\0';
    for (const unsigned char c : uncommented) {
        if (quote == '\0' && (c == '\'' || c == '"')) {
            quote = static_cast<char>(c);
            normalized.push_back(static_cast<char>(c));
        } else if (quote != '\0' && c == static_cast<unsigned char>(quote)) {
            quote = '\0';
            normalized.push_back(static_cast<char>(c));
        } else if (quote != '\0' || !std::isspace(c)) {
            normalized.push_back(static_cast<char>(c));
        }
    }
    return normalized;
}

struct CheckResult
{
    std::string name;
    bool        pass {false};
    std::string detail;
};

void report(std::vector<CheckResult>& results, const std::string& name, bool pass, const std::string& detail)
{
    results.push_back({name, pass, detail});
    std::cout << (pass ? "  [PASS] " : "  [FAIL] ") << name << "  " << detail << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const std::string urdf_path   = argc > 1 ? argv[1] : "project/robot/models/bodys/wheel_leg_robot.xacro";
        const std::string output_path = argc > 2 ? argv[2] : "project/params/leg_gain/nominal.yaml";
        const std::string params_path = argc > 3 ? argv[3] : "project/params/chassis.yaml";
        const std::string side        = "left";
        constexpr double  gravity     = 9.81;
        constexpr double  dt          = 0.001;
        constexpr double  effort_limit = 1.5;   // 模型里的关节硬限位，工作点检查用

        std::cout << "generate_wheel_leg_lqr\n";
        std::cout << "  URDF   : " << urdf_path << "\n";
        std::cout << "  增益   : " << output_path << "\n";
        std::cout << "  参数   : " << params_path << "（Q/R）\n";

        const std::string urdf_text = load_urdf_text(urdf_path);
        const Urdf urdf = parse_urdf(urdf_text);
        PlanarRobot robot = build_planar_robot(urdf, side);

        // 调高保持轮心仍在髋轴正下方，因此 q_k=-2q_h；高度参数缺省时就是 URDF 零位。
        const double link_length = std::hypot(robot.joint[1].origin_x, robot.joint[1].origin_z);
        const double link_angle = std::atan2(-robot.joint[1].origin_z, robot.joint[1].origin_x);
        const double nominal_height = 2.0 * link_length * std::sin(link_angle);
        const double operating_height = argc > 4 ? std::stod(argv[4]) : nominal_height;
        if (!std::isfinite(operating_height) || operating_height <= 0.0 ||
            operating_height >= 2.0 * link_length)
        {
            throw std::invalid_argument("腿高工作点超出几何可达范围");
        }
        const double operating_hip = std::asin(operating_height / (2.0 * link_length)) - link_angle;
        const double operating_knee = -2.0 * operating_hip;
        GeneralVector q0 = GeneralVector::Zero();
        q0(kQh) = operating_hip;
        q0(kQk) = operating_knee;
        std::cout << "  工作点 : h=" << operating_height << " m，q_h=" << operating_hip
                  << " rad，q_k=" << operating_knee << " rad\n";

        std::vector<CheckResult> checks;

        // ---- 左右对称：两条腿的质量、惯量、几何逐项比 ----
        {
            const std::string other = (side == "left") ? "right" : "left";
            double worst = 0.0;
            auto compare = [&](double a, double b) { worst = std::max(worst, std::abs(a - b)); };

            for (const std::string& suffix : {std::string("_thigh_link"), std::string("_calf_link"),
                                              std::string("_wheel_link"), std::string("_hip_link"),
                                              std::string("_knee_link")}) {
                const LinkInfo& l = urdf.link(side + suffix);
                const LinkInfo& r = urdf.link(other + suffix);
                compare(l.mass, r.mass);
                compare(l.iyy, r.iyy);
                compare(std::abs(l.com_x), std::abs(r.com_x));
                compare(std::abs(l.com_z), std::abs(r.com_z));
            }
            for (const std::string& suffix : {std::string("_hip_joint"), std::string("_knee_joint"),
                                              std::string("_wheel_joint"), std::string("_knee_cylinder_joint"),
                                              std::string("_hip_cylinder_joint")}) {
                const JointInfo& l = urdf.joint(side + suffix);
                const JointInfo& r = urdf.joint(other + suffix);
                compare(std::abs(l.origin_x), std::abs(r.origin_x));
                compare(std::abs(l.origin_z), std::abs(r.origin_z));
                compare(l.axis_y, r.axis_y);
            }
            std::ostringstream detail;
            detail << "左右逐项最大差 " << std::scientific << std::setprecision(2) << worst;
            report(checks, "左右对称", worst < 1e-12, detail.str());
        }

        // ---- 冻结腿姿的等效倒立摆参数：拿"机身+两条腿"的合成质心算出来 ----
        {
            const auto kin = kinematics(robot, q0);

            double mass_sum = 0.0;
            double mz = 0.0;
            double mx = 0.0;
            for (int i = kBody; i <= kCalf; ++i) {          // 机身 + 大腿 + 小腿
                mass_sum += robot.body[i].mass;
                mz += robot.body[i].mass * kin[i].com_z;
                mx += robot.body[i].mass * kin[i].com_x;
            }
            // l 是"到轮轴"的竖直距离，不是世界高度：kBody..kCalf 都在世界系里，
            // 那个系的零点在地面，轮轴在 r 处，所以要减掉 r
            robot.locked_com_height = mz / mass_sum - robot.wheel_radius;
            const double com_x = mx / mass_sum;

            double inertia = 0.0;
            for (int i = kBody; i <= kCalf; ++i) {
                const double dx = kin[i].com_x - com_x;
                // kin 的 z 是世界高度（零点在地面），locked_com_height 是到轮轴的，
                // 两个口径不能混 —— 混了平行轴项会白白多出 r² 那一档
                const double dz = (kin[i].com_z - robot.wheel_radius) - robot.locked_com_height;
                inertia += robot.body[i].inertia_yy + robot.body[i].mass * (dx * dx + dz * dz);
            }
            robot.locked_pitch_inertia = inertia;

            std::cout << "\n[模型摘要]\n";
            std::cout << std::fixed << std::setprecision(5);
            std::cout << "  r = " << robot.wheel_radius
                      << "  span = " << robot.joint[1].origin_x
                      << "  drop = " << -robot.joint[1].origin_z << "\n";
            std::cout << "  质量 机身 " << robot.body[kBody].mass
                      << " / 大腿 " << robot.body[kThigh].mass
                      << " / 小腿 " << robot.body[kCalf].mass
                      << " / 轮 " << robot.body[kWheel].mass
                      << " / 合计 " << robot.body[kBody].mass + robot.body[kThigh].mass + robot.body[kCalf].mass + robot.body[kWheel].mass << "\n";
            for (int i = 0; i < 4; ++i) {
                std::cout << "    " << robot.body[i].name << "：m " << robot.body[i].mass
                          << "  COM(" << robot.body[i].com_x << ", " << robot.body[i].com_z << ")"
                          << "  Iyy " << robot.body[i].inertia_yy << "\n";
            }
            std::cout << "  冻结腿姿等效倒立摆：M " << robot.cart_mass
                      << "  m " << mass_sum
                      << "  l " << robot.locked_com_height
                      << "  I " << robot.locked_pitch_inertia << "\n";
        }

        // ---- 工作点 ----
        const GeneralVector g0 = gravity_vector(robot, q0, gravity);
        const GeneralInput  s  = input_matrix(robot);

        // G(q0) = Sᵀ U0：s 行（水平平衡）定 τ_w，髋/膝行定 τ_h / τ_k。
        // θ 那行没有哪个执行器能补 —— 轮力矩的反作用已经算在 S 里了 —— 剩下的就是
        // 整车组合质心不在轮轴正上方造成的静态力矩
        const double u0_wheel = g0(kS) * robot.wheel_radius / 2.0;
        const double u0_hip   = (g0(kQh) + 2.0 * u0_wheel) / 2.0;
        const double u0_knee  = (g0(kQk) + 2.0 * u0_wheel) / 2.0;

        // 俯仰方向的残差：整机重量乘组合质心的水平偏移。标准 V 形腿的两根杆都在轮轴
        // 前方，所以质心偏前、这个残差不为零 —— 它是几何事实，不是算错
        const auto kin0 = kinematics(robot, q0);
        double total_mass = 0.0;
        double total_mx = 0.0;
        for (int i = 0; i < 4; ++i) {
            total_mass += robot.body[i].mass;
            total_mx   += robot.body[i].mass * kin0[i].com_x;
        }
        const double total_com_x  = total_mx / total_mass;
        const double pitch_moment = -total_mass * gravity * total_com_x;
        const double pitch_residual = g0(kTh) - s(kS, kTauWheel) * 0.0 - (-2.0 * u0_wheel);

        std::cout << "\n[工作点]\n";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  G(q0) = [" << g0(kS) << ", " << g0(kTh) << ", " << g0(kQh) << ", " << g0(kQk) << "]\n";
        std::cout << "  U0 = [tau_w " << u0_wheel << ", tau_h " << u0_hip << ", tau_k " << u0_knee << "]\n";
        std::cout << "  整车组合质心 x = " << total_com_x << " m（前偏）→ 俯仰残差 "
                  << pitch_residual << " N·m（= −m·g·x，期望 " << pitch_moment << "）\n";

        report(checks, "俯仰残差 = 质心偏移力矩", std::abs(pitch_residual - pitch_moment) < 1e-6,
               "残差 " + std::to_string(pitch_residual) + "，−m·g·x = " + std::to_string(pitch_moment));
        report(checks, "U0 在力矩上限内", std::max({std::abs(u0_wheel), std::abs(u0_hip), std::abs(u0_knee)}) < effort_limit,
               "max |U0| = " + std::to_string(std::max({std::abs(u0_wheel), std::abs(u0_hip), std::abs(u0_knee)})));
        const bool nominal_operating_point = std::abs(operating_height - nominal_height) < 1e-10;
        report(checks, "U0 与标称静力参考一致",
               !nominal_operating_point ||
                   (std::abs(u0_hip + 0.0147) < 5e-4 && std::abs(u0_knee + 0.2567) < 5e-4),
               nominal_operating_point
                   ? "胯 " + std::to_string(u0_hip) + "（期望 -0.0147），膝 " +
                         std::to_string(u0_knee) + "（期望 -0.2567）"
                   : "非零高度工作点，U0 由该姿态的 G(q0) 直接求解");

        // ---- A/B：三档步长看漂不漂 ----
        const std::array<double, 3> steps{1e-4, 1e-5, 1e-6};
        std::array<StateMatrix, 3> a_set{};
        std::array<InputMatrix, 3> b_set{};
        for (std::size_t i = 0; i < steps.size(); ++i) {
            std::tie(a_set[i], b_set[i]) = linearize(robot, q0, gravity, steps[i]);
        }

        double worst_a = 0.0;
        double worst_b = 0.0;
        for (std::size_t i = 1; i < steps.size(); ++i) {
            worst_a = std::max(worst_a, (a_set[i] - a_set[0]).cwiseAbs().maxCoeff());
            worst_b = std::max(worst_b, (b_set[i] - b_set[0]).cwiseAbs().maxCoeff());
        }
        // 判据用相对量：A 的元素本身几百，拿绝对 1e-6 当门槛等于在量差分噪声。
        // G 现在是解析的，只剩 ∂G/∂q 这一层差分，相对差应该很小
        const double scale_a = a_set[1].cwiseAbs().maxCoeff();
        const double scale_b = std::max(b_set[1].cwiseAbs().maxCoeff(), 1e-12);
        report(checks, "差分步长扫描稳定", worst_a < 1e-6 * scale_a && worst_b < 1e-6 * scale_b,
               "步长变一个数量级：A 相对差 " + std::to_string(worst_a / scale_a) + "，B 相对差 " + std::to_string(worst_b / scale_b));

        const StateMatrix a = a_set[1];
        const InputMatrix b = b_set[1];

        // ---- 权重：从参数文件取 q_pitch / r_pitch ----
        const std::string params_text = read_file(params_path);
        StateMatrix q = StateMatrix::Zero();
        for (std::size_t i = 0; i < kStateDim; ++i) {
            q(i, i) = parse_list_entry(params_text, "q_pitch", i, 1.0);
        }
        ControlMatrix r = ControlMatrix::Zero();
        for (std::size_t i = 0; i < kInputDim; ++i) {
            r(i, i) = parse_list_entry(params_text, "r_pitch", i, 1.0);
        }

        // ---- 离散 + 求 K ----
        const auto [ad, bd] = discretize(a, b, dt);

        algorithm::controller::Lqr<kStateDim, kInputDim, double> lqr;
        lqr.configure(ad, bd, q, r);
        lqr.solve(100000, 1e-12);
        const auto k = lqr.gain();

        std::cout << "\n[A_pitch]\n" << a << "\n";
        std::cout << "\n[B_pitch]\n" << b << "\n";
        std::cout << "\n[K_pitch]\n" << k << "\n";

        std::array<double, kInputDim> pitch_ablation_radius{};
        for (std::size_t disabled = 0; disabled < kInputDim; ++disabled) {
            InputMatrix disabled_bd = bd;
            disabled_bd.col(static_cast<Eigen::Index>(disabled)).setZero();
            const auto eigenvalues = (ad - disabled_bd * k).eigenvalues();
            for (int i = 0; i < eigenvalues.size(); ++i) {
                pitch_ablation_radius[disabled] = std::max(pitch_ablation_radius[disabled], std::abs(eigenvalues(i)));
            }
        }
        std::cout << "[Pitch 执行器禁用预测] max |eig|：轮/髋/膝 = "
                  << pitch_ablation_radius[0] << " / " << pitch_ablation_radius[1] << " / "
                  << pitch_ablation_radius[2] << "\n";

        // ---- 检查 ----
        std::cout << "\n[检查]\n";

        report(checks, "维度", a.rows() == kStateDim && a.cols() == kStateDim && b.rows() == kStateDim && b.cols() == kInputDim,
               "A 8x8、B 8x3");
        report(checks, "有限性", all_finite(a) && all_finite(b) && all_finite(q) && all_finite(r) && all_finite(k),
               "A/B/Q/R/K 无 NaN、Inf");

        const double q_sym = (q - q.transpose()).cwiseAbs().maxCoeff();
        const bool q_psd = Eigen::SelfAdjointEigenSolver<StateMatrix>(q).eigenvalues().minCoeff() > -1e-9;
        report(checks, "Q 对称半正定", q_sym < 1e-12 && q_psd, "|Q-Qᵀ|max = " + std::to_string(q_sym));

        const double r_sym = (r - r.transpose()).cwiseAbs().maxCoeff();
        const bool r_pd = Eigen::SelfAdjointEigenSolver<ControlMatrix>(r).eigenvalues().minCoeff() > 1e-12;
        report(checks, "R 对称正定", r_sym < 1e-12 && r_pd, "|R-Rᵀ|max = " + std::to_string(r_sym));

        Eigen::MatrixXd controllability(kStateDim, kStateDim * kInputDim);
        Eigen::MatrixXd power = b;
        for (std::size_t i = 0; i < kStateDim; ++i) {
            controllability.block(0, static_cast<int>(i) * static_cast<int>(kInputDim), static_cast<int>(kStateDim), static_cast<int>(kInputDim)) = power;
            power = a * power;
        }
        const int rank = rank_of(controllability, 1e-9);
        report(checks, "可控性", rank == static_cast<int>(kStateDim),
               "rank([B AB ... A^7 B]) = " + std::to_string(rank) + " / 8");

        const auto open_loop = a.eigenvalues();
        double worst_open = -1e9;
        for (int i = 0; i < open_loop.size(); ++i) {
            worst_open = std::max(worst_open, open_loop(i).real());
        }
        report(checks, "开环有倒立不稳定模态", worst_open > 1e-3,
               "max Re(eig) = " + std::to_string(worst_open));

        const StateMatrix closed = ad - bd * k;
        const auto closed_loop = closed.eigenvalues();
        double worst_closed = 0.0;
        for (int i = 0; i < closed_loop.size(); ++i) {
            worst_closed = std::max(worst_closed, std::abs(closed_loop(i)));
        }
        report(checks, "闭环全在单位圆内", worst_closed < 1.0,
               "max |eig(Ad-BdK)| = " + std::to_string(worst_closed));

        // ---- 锁腿退化对照：跟 chassis 现有的 4 状态倒立摆模型比 ----
        {
            const double m = robot.locked_mass;
            const double l = robot.locked_com_height;
            const double i = robot.locked_pitch_inertia;
            const double big_m = robot.cart_mass;
            const double p = i * (big_m + m) + big_m * m * l * l;

            // 本模型的退化版：腿坐标是约束（q̈_腿 = 0），只留 [s, θ]
            const GeneralMatrix m_full = mass_matrix(robot, q0);
            const GeneralMatrix m_inv = m_full.inverse();

            Eigen::Matrix2d m11 = Eigen::Matrix2d::Zero();
            m11(0, 0) = m_full(kS, kS);
            m11(0, 1) = m_full(kS, kTh);
            m11(1, 0) = m_full(kTh, kS);
            m11(1, 1) = m_full(kTh, kTh);
            const Eigen::Matrix2d m11_inv = m11.inverse();

            GeneralMatrix dg = GeneralMatrix::Zero();
            for (std::size_t j = 0; j < kGeneralDim; ++j) {
                GeneralVector q_plus  = q0;
                GeneralVector q_minus = q0;
                q_plus(j)  += 1e-6;
                q_minus(j) -= 1e-6;
                dg.col(j) = (gravity_vector(robot, q_plus, gravity) - gravity_vector(robot, q_minus, gravity)) / 2e-6;
            }

            Eigen::Matrix2d dg11 = Eigen::Matrix2d::Zero();
            dg11(0, 0) = dg(kS, kS);
            dg11(0, 1) = dg(kS, kTh);
            dg11(1, 0) = dg(kTh, kS);
            dg11(1, 1) = dg(kTh, kTh);

            Eigen::Matrix2d a_state = -m11_inv * dg11;
            Eigen::Matrix2d a4_tool = Eigen::Matrix2d::Zero();
            a4_tool(0, 0) = a_state(0, 0);
            a4_tool(0, 1) = a_state(0, 1);
            a4_tool(1, 0) = a_state(1, 0);
            a4_tool(1, 1) = a_state(1, 1);

            // 输入的广义力：轮力矩在 s 行是 2τ/r，在 θ 行是它的反作用 −2τ
            Eigen::Vector2d b4_tool = m11_inv * Eigen::Vector2d(2.0 / robot.wheel_radius, -2.0);

            // chassis 那套（chassis.hpp configure_lqr()，阻尼 b = 0：本模型没有阻尼）：
            // 只比加速度那两行 —— 本模型的 a_state = ∂(s̈, θ̈)/∂(s, θ)。
            // 下标别对错：chassis 的 continuous_a(1,2) 是 ∂s̈/∂θ，落到这里就是 (0,1)
            Eigen::Matrix2d a_chassis_acc = Eigen::Matrix2d::Zero();
            a_chassis_acc(0, 1) = -m * m * gravity * l * l / p;
            a_chassis_acc(1, 1) =  m * gravity * l * (big_m + m) / p;
            const Eigen::Vector2d b_chassis =
                Eigen::Vector2d(2.0 * ((i + m * l * l) / robot.wheel_radius + m * l) / p,
                                -2.0 * (m * l / robot.wheel_radius + big_m + m) / p);

            // 判据用相对量、给到 1%：两套模型的降阶方式不一样 —— 工具是全 4 体平面模型
            // （腿的连杆各有自己的惯量），chassis 是 m/l/I 三参数集总的倒立摆
            // （把轮子的自转惯量当成纯平动质量）。剩下 ~0.1% 的差是这个口径差，
            // 不是错；真出错（比如轮力矩漏了反作用）是 20% 起的量级
            const double diff_a = (a_state - a_chassis_acc).cwiseAbs().maxCoeff();
            const double diff_b = (b4_tool - b_chassis).cwiseAbs().maxCoeff();
            const double rel_a = diff_a / a_chassis_acc.cwiseAbs().maxCoeff();
            const double rel_b = diff_b / b_chassis.cwiseAbs().maxCoeff();
            std::ostringstream detail;
            detail << std::scientific << std::setprecision(3)
                   << "∂(s̈,θ̈)/∂(s,θ) 相对差 " << rel_a << "，B 相对差 " << rel_b;
            report(checks, "锁腿退化对照（现有 4 状态模型）", rel_a < 1e-2 && rel_b < 1e-2, detail.str());

            std::cout << "   工具  a_state = \n" << a_state << "\n   工具  b = " << b4_tool.transpose() << "\n";
            std::cout << "   底盘  acc     = \n" << a_chassis_acc << "\n   底盘  b = " << b_chassis.transpose() << "\n";
        }

        // ---- Roll（差模）子系统：4 状态 [q_h−, q̇_h−, q_k−, q̇_k−]，输入 [τ_h−, τ_k−] ----
        FrontalRobot frontal = build_frontal_robot(urdf, side, robot);
        frontal.common_hip = operating_hip;
        frontal.common_knee = operating_knee;

        const RollGeneralVector roll_zero = RollGeneralVector::Zero();
        const RollGeneralVector roll_g0   = frontal_gravity_vector(frontal, roll_zero, gravity);
        const double u0_roll_h = roll_g0(0) / 2.0;
        const double u0_roll_k = roll_g0(1) / 2.0;

        std::array<RollStateMatrix, 3> roll_a_set{};
        std::array<RollInputMatrix, 3> roll_b_set{};
        for (std::size_t i = 0; i < steps.size(); ++i) {
            std::tie(roll_a_set[i], roll_b_set[i]) = frontal_linearize(frontal, roll_zero, gravity, steps[i]);
        }
        double worst_roll_a = 0.0;
        double worst_roll_b = 0.0;
        for (std::size_t i = 1; i < steps.size(); ++i) {
            worst_roll_a = std::max(worst_roll_a, (roll_a_set[i] - roll_a_set[0]).cwiseAbs().maxCoeff());
            worst_roll_b = std::max(worst_roll_b, (roll_b_set[i] - roll_b_set[0]).cwiseAbs().maxCoeff());
        }

        const RollStateMatrix a_roll = roll_a_set[1];
        const RollInputMatrix b_roll = roll_b_set[1];
        const auto [ad_roll, bd_roll] = discretize(a_roll, b_roll, dt);

        // Roll 权重：参数文件里那 6 个是按 [roll, φ̇, q_h−, q̇_h−, q_k−, q̇_k−] 给的，
        // 但本模型里 roll 是被腿差模决定的（不是独立状态），所以把 roll / φ̇ 那两份
        // 通过 ∂φ/∂q 折进腿差模的二次型里 —— 等价于"惩罚 roll"，只是换到 4 状态坐标系
        RollControlMatrix r_roll = RollControlMatrix::Zero();
        for (std::size_t i = 0; i < kRollInputDim; ++i) {
            r_roll(i, i) = parse_list_entry(params_text, "r_roll", i, 1.0);
        }

        const double q_roll_weight[6] = {
            parse_list_entry(params_text, "q_roll", 0, 800.0), parse_list_entry(params_text, "q_roll", 1, 60.0),
            parse_list_entry(params_text, "q_roll", 2, 400.0), parse_list_entry(params_text, "q_roll", 3, 40.0),
            parse_list_entry(params_text, "q_roll", 4, 400.0), parse_list_entry(params_text, "q_roll", 5, 40.0)};

        Eigen::MatrixXd roll_jacobian = Eigen::MatrixXd::Zero(1, kRollGeneralDim);
        {
            constexpr double step = 1e-7;
            for (std::size_t j = 0; j < kRollGeneralDim; ++j) {
                RollGeneralVector plus  = roll_zero;
                RollGeneralVector minus = roll_zero;
                plus(j)  += step;
                minus(j) -= step;
                roll_jacobian(0, j) = (frontal_kinematics(frontal, plus).roll
                                     - frontal_kinematics(frontal, minus).roll) / (2.0 * step);
            }
        }
        // 位置那份（roll）挂在第 0、2 个状态上，速度那份（φ̇）挂在第 1、3 个上
        Eigen::MatrixXd to_position = Eigen::MatrixXd::Zero(kRollGeneralDim, kRollStateDim);
        Eigen::MatrixXd to_velocity = Eigen::MatrixXd::Zero(kRollGeneralDim, kRollStateDim);
        for (std::size_t i = 0; i < kRollGeneralDim; ++i) {
            to_position(i, 2 * i)     = 1.0;
            to_velocity(i, 2 * i + 1) = 1.0;
        }

        RollStateMatrix q_roll = RollStateMatrix::Zero();
        for (std::size_t i = 0; i < kRollGeneralDim; ++i) {          // 腿差模自己那四个权重
            q_roll(2 * i, 2 * i)         = q_roll_weight[2 + 2 * i];
            q_roll(2 * i + 1, 2 * i + 1) = q_roll_weight[3 + 2 * i];
        }
        q_roll += q_roll_weight[0] * to_position.transpose() * roll_jacobian.transpose() * roll_jacobian * to_position;
        q_roll += q_roll_weight[1] * to_velocity.transpose() * roll_jacobian.transpose() * roll_jacobian * to_velocity;

        algorithm::controller::Lqr<kRollStateDim, kRollInputDim, double> roll_lqr;
        roll_lqr.configure(ad_roll, bd_roll, q_roll, r_roll);
        roll_lqr.solve(100000, 1e-12);
        const auto k_roll = roll_lqr.gain();

        std::array<double, kRollInputDim> roll_ablation_radius{};
        for (std::size_t disabled = 0; disabled < kRollInputDim; ++disabled) {
            RollInputMatrix disabled_bd = bd_roll;
            disabled_bd.col(static_cast<Eigen::Index>(disabled)).setZero();
            const auto eigenvalues = (ad_roll - disabled_bd * k_roll).eigenvalues();
            for (int i = 0; i < eigenvalues.size(); ++i) {
                roll_ablation_radius[disabled] = std::max(roll_ablation_radius[disabled], std::abs(eigenvalues(i)));
            }
        }

        std::cout << "\n[Roll 子系统]\n";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  G_r(q0) = [" << roll_g0(0) << ", " << roll_g0(1) << "]  U0_roll = ["
                  << u0_roll_h << ", " << u0_roll_k << "]\n";
        std::cout << "  ∂φ/∂q = [" << roll_jacobian(0, 0) << ", " << roll_jacobian(0, 1) << "]\n";
        std::cout << "\n[A_roll]\n" << a_roll << "\n\n[B_roll]\n" << b_roll << "\n\n[K_roll]\n" << k_roll << "\n";
        std::cout << "[Roll 执行器禁用预测] max |eig|：髋/膝 = "
                  << roll_ablation_radius[0] << " / " << roll_ablation_radius[1] << "\n";

        // 几何一致性：随便取一个非零差模角，右侧那条腿的解应该跟左侧推出来的一致
        double worst_residual = 0.0;
        for (double test : {0.02, 0.05, -0.03, -0.06}) {
            RollGeneralVector rr = RollGeneralVector::Zero();
            rr(0) = test;
            rr(1) = -0.5 * test;
            worst_residual = std::max(worst_residual, frontal_kinematics(frontal, rr).constraint_residual);
        }
        report(checks, "Roll 几何自相一致（左右两条腿给同一个机身原点）", worst_residual < 1e-12,
               "非零差模角下残差 " + std::to_string(worst_residual));

        report(checks, "Roll 左右对称工作点（差模前馈为零）",
               std::abs(u0_roll_h) < 1e-9 && std::abs(u0_roll_k) < 1e-9,
               "U0_roll = [" + std::to_string(u0_roll_h) + ", " + std::to_string(u0_roll_k) + "]");

        {
            const double scale = std::max(roll_a_set[1].cwiseAbs().maxCoeff(), 1e-12);
            const double scale_b = std::max(roll_b_set[1].cwiseAbs().maxCoeff(), 1e-12);
            double gravity_error = 0.0;
            double gravity_scale = 1.0;
            constexpr double verify_step = 1e-6;
            for (const RollGeneralVector sample : {
                     RollGeneralVector(0.0, 0.0), RollGeneralVector(0.02, -0.01), RollGeneralVector(-0.04, 0.02)}) {
                const RollGeneralVector analytic = frontal_gravity_vector(frontal, sample, gravity);
                RollGeneralVector numeric = RollGeneralVector::Zero();
                for (std::size_t j = 0; j < kRollGeneralDim; ++j) {
                    RollGeneralVector plus = sample;
                    RollGeneralVector minus = sample;
                    plus(j) += verify_step;
                    minus(j) -= verify_step;
                    numeric(j) = (frontal_potential(frontal, plus, gravity)
                                - frontal_potential(frontal, minus, gravity)) / (2.0 * verify_step);
                }
                gravity_error = std::max(gravity_error, (analytic - numeric).cwiseAbs().maxCoeff());
                gravity_scale = std::max(gravity_scale, analytic.cwiseAbs().maxCoeff());
            }
            const double gravity_relative_error = gravity_error / gravity_scale;
            report(checks, "Roll 差分步长扫描稳定",
                   worst_roll_a < 1e-5 * scale && worst_roll_b < 1e-5 * scale_b && gravity_relative_error < 1e-7,
                   "A 相对差 " + std::to_string(worst_roll_a / scale)
                   + "，B 相对差 " + std::to_string(worst_roll_b / scale_b)
                   + "，G 解析/能量差分相对差 " + std::to_string(gravity_relative_error));
        }
        report(checks, "Roll 有限性",
               all_finite(a_roll) && all_finite(b_roll) && all_finite(k_roll), "A/B/K 无 NaN、Inf");

        {
            Eigen::MatrixXd controllability(kRollStateDim, kRollStateDim * kRollInputDim);
            Eigen::MatrixXd power = b_roll;
            for (std::size_t i = 0; i < kRollStateDim; ++i) {
                controllability.block(0, static_cast<int>(i) * static_cast<int>(kRollInputDim),
                                      static_cast<int>(kRollStateDim), static_cast<int>(kRollInputDim)) = power;
                power = a_roll * power;
            }
            const int rank = rank_of(controllability, 1e-9);
            report(checks, "Roll 可控性", rank == static_cast<int>(kRollStateDim),
                   "rank = " + std::to_string(rank) + " / 4");
        }

        {
            const auto open_loop = a_roll.eigenvalues();
            double worst_open = -1e9;
            for (int i = 0; i < open_loop.size(); ++i) {
                worst_open = std::max(worst_open, open_loop(i).real());
            }
            report(checks, "Roll 开环有倒立不稳定模态", worst_open > 1e-3,
                   "max Re(eig) = " + std::to_string(worst_open));

            const auto closed_loop = (ad_roll - bd_roll * k_roll).eigenvalues();
            double worst_closed = 0.0;
            for (int i = 0; i < closed_loop.size(); ++i) {
                worst_closed = std::max(worst_closed, std::abs(closed_loop(i)));
            }
            report(checks, "Roll 闭环全在单位圆内", worst_closed < 1.0,
                   "max |eig(Ad-BdK)| = " + std::to_string(worst_closed));
        }

        // 轮差动力矩不进 Roll 子系统：本模型里它是偏航自由度上的输入（不含偏航），
        // B_roll 的两列只有髋/膝差模 —— 这与文档 §3.3 的判断一致，τ_w− 继续留给偏航外环
        report(checks, "Roll 输入只含髋/膝差模（轮差动留给偏航外环）", kRollInputDim == 2,
               "B_roll 只有 tau_h_diff / tau_k_diff 两列");

        // ---- 写增益文件 ----
        {
            std::ostringstream os;
            os << std::setprecision(12);
            os << "# 由 generate_wheel_leg_lqr 从 URDF 生成，别手改（改模型就重跑）\n";
            os << "# 依据：docs/轮腿机器人平面模型参数与公式.md\n";
            os << "model:\n";
            os << "  urdf: " << urdf_path << "\n";
            os << "  hash: \"0x" << std::hex << fnv1a(model_hash_text(urdf_text)) << std::dec << "\"\n";
            os << "  side: " << side << "\n";
            os << "  wheel_radius: " << robot.wheel_radius << "\n";
            os << "  dt: " << dt << "\n";
            os << "  gravity: " << gravity << "\n";
            os << "  mass:\n";
            os << "    body: " << robot.body[kBody].mass << "\n";
            os << "    thigh: " << robot.body[kThigh].mass << "\n";
            os << "    calf: " << robot.body[kCalf].mass << "\n";
            os << "    wheel: " << robot.body[kWheel].mass << "\n";
            os << "  geometry:\n";
            os << "    hip_origin: [" << robot.joint[0].origin_x << ", " << robot.joint[0].origin_z << "]\n";
            os << "    knee_origin: [" << robot.joint[1].origin_x << ", " << robot.joint[1].origin_z << "]\n";
            os << "    wheel_origin: [" << robot.joint[2].origin_x << ", " << robot.joint[2].origin_z << "]\n";
            os << "  locked_pendulum:\n";
            os << "    cart_mass: " << robot.cart_mass << "\n";
            os << "    body_mass: " << robot.locked_mass << "\n";
            os << "    com_height: " << robot.locked_com_height << "\n";
            os << "    pitch_inertia: " << robot.locked_pitch_inertia << "\n";

            StateVector x0 = StateVector::Zero();
            x0(4) = operating_hip;
            x0(6) = operating_knee;
            Eigen::Matrix<double, kInputDim, 1> u0;
            u0 << u0_wheel, u0_hip, u0_knee;
            os << "operating_point:\n";
            os << "  height: " << operating_height << "\n";
            os << "  x0: " << vector_to_yaml(x0) << "\n";
            os << "  u0: " << vector_to_yaml(u0) << "\n";

            os << "layout:\n";
            os << "  states: [";
            for (std::size_t i = 0; i < kStateDim; ++i) {
                os << (i == 0 ? "" : ", ") << kStateNames[i];
            }
            os << "]\n  inputs: [";
            for (std::size_t i = 0; i < kInputDim; ++i) {
                os << (i == 0 ? "" : ", ") << kInputNames[i];
            }
            os << "]\n";

            os << "continuous:\n";
            os << "  a: " << matrix_to_yaml(a) << "\n";
            os << "  b: " << matrix_to_yaml(b) << "\n";
            os << "discrete:\n";
            os << "  ad: " << matrix_to_yaml(ad) << "\n";
            os << "  bd: " << matrix_to_yaml(bd) << "\n";
            os << "gain:\n";
            os << "  k: " << matrix_to_yaml(k) << "\n";
            os << "weights:\n";
            os << "  q: " << vector_to_yaml(q.diagonal()) << "\n";
            os << "  r: " << vector_to_yaml(r.diagonal()) << "\n";
            os << "ablation_prediction:\n";
            os << "  pitch_max_eigenvalue_without: {wheel_common: " << pitch_ablation_radius[0]
               << ", hip_common: " << pitch_ablation_radius[1]
               << ", knee_common: " << pitch_ablation_radius[2] << "}\n";

            // Roll（差模）子系统：4 状态 [q_h−, q̇_h−, q_k−, q̇_k−]，2 输入。
            // roll 本身不是状态：两轮贴地时它由左右腿高差唯一决定（见工具头注释）
            os << "roll:\n";
            os << "  layout:\n";
            os << "    states: [";
            for (std::size_t i = 0; i < kRollStateDim; ++i) {
                os << (i == 0 ? "" : ", ") << kRollStateNames[i];
            }
            os << "]\n    inputs: [";
            for (std::size_t i = 0; i < kRollInputDim; ++i) {
                os << (i == 0 ? "" : ", ") << kRollInputNames[i];
            }
            os << "]\n";
            os << "  operating_point:\n";
            os << "    x0: [0, 0, 0, 0]\n";
            os << "    u0: [" << u0_roll_h << ", " << u0_roll_k << "]\n";
            os << "    roll_per_q: [" << roll_jacobian(0, 0) << ", " << roll_jacobian(0, 1) << "]\n";
            os << "  continuous:\n";
            os << "    a: " << matrix_to_yaml(a_roll) << "\n";
            os << "    b: " << matrix_to_yaml(b_roll) << "\n";
            os << "  discrete:\n";
            os << "    ad: " << matrix_to_yaml(ad_roll) << "\n";
            os << "    bd: " << matrix_to_yaml(bd_roll) << "\n";
            os << "  gain:\n";
            os << "    k: " << matrix_to_yaml(k_roll) << "\n";
            os << "  weights:\n";
            os << "    q: " << vector_to_yaml(q_roll.diagonal()) << "\n";
            os << "    r: " << vector_to_yaml(r_roll.diagonal()) << "\n";
            os << "  ablation_prediction:\n";
            os << "    max_eigenvalue_without: {hip_diff: " << roll_ablation_radius[0]
               << ", knee_diff: " << roll_ablation_radius[1] << "}\n";

            os << "checks:\n";
            bool all_pass = true;
            for (const auto& c : checks) {
                all_pass = all_pass && c.pass;
                os << "  " << c.name << ": " << (c.pass ? "pass" : "fail") << "\n";
            }

            std::ofstream out(output_path);
            if (!out) {
                throw std::runtime_error("写不了增益文件：" + output_path);
            }
            out << os.str();
            std::cout << "\n[输出] " << output_path << "\n";

            std::size_t failed = 0;
            for (const auto& c : checks) {
                failed += c.pass ? 0 : 1;
            }
            std::cout << "\n" << (failed == 0 ? "全部检查通过" : std::to_string(failed) + " 项检查没过")
                      << "（共 " << checks.size() << " 项）\n";
            return failed == 0 ? 0 : 1;
        }
    } catch (const std::exception& error) {
        std::cerr << "出错：" << error.what() << "\n";
        return 2;
    }
}
