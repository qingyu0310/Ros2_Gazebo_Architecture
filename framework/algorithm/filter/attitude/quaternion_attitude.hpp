/**
 * @file quaternion_attitude.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-09-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace algorithm::filter {

template <typename Scalar = double>
class QuaternionAttitude {
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Quaternion = Eigen::Quaternion<Scalar>;

    QuaternionAttitude() = default;

    explicit QuaternionAttitude(const Quaternion& initial) : q_(initial.normalized()) {}

    void reset(const Quaternion& q = Quaternion::Identity()) { q_ = q.normalized(); }

    void set_accel_correction_gain(Scalar gain) {
        accel_correction_gain_ = std::max(Scalar(0), gain);
    }

    void predict(const Vector3& gyro_radps, Scalar dt_s) {
        if (dt_s <= Scalar(0)) {
            return;
        }

        const Scalar angle = gyro_radps.norm() * dt_s;
        if (angle <= Scalar(0)) {
            return;
        }

        const Vector3 axis = gyro_radps.normalized();
        q_ = (q_ * Quaternion(Eigen::AngleAxis<Scalar>(angle, axis))).normalized();
    }

    void correct_accel(const Vector3& accel_mps2) {
        const Scalar norm = accel_mps2.norm();
        if (norm <= Scalar(0)) {
            return;
        }

        // 加速度计静止时读的是比力 +g（沿世界天向），所以这两个向量的物理含义都是"天向"
        // 在机体系里的分量，姿态对时两者相同。叉乘顺序就是反馈符号，别调过来：
        // estimated × measured 是正反馈，姿态会一路翻到 ±180° 才停在那儿（那里叉乘也是 0）。
        const Vector3 measured_up = accel_mps2 / norm;
        const Vector3 estimated_up = q_.inverse() * world_up();
        const Vector3 error = measured_up.cross(estimated_up);

        if (error.norm() <= Scalar(0)) {
            return;
        }

        const Vector3 correction = accel_correction_gain_ * error;
        const Scalar angle = correction.norm();
        if (angle <= Scalar(0)) {
            return;
        }

        q_ = (q_ * Quaternion(Eigen::AngleAxis<Scalar>(angle, correction / angle))).normalized();
    }

    void update(const Vector3& gyro_radps, const Vector3& accel_mps2, Scalar dt_s) {
        predict(gyro_radps, dt_s);
        correct_accel(accel_mps2);
    }

    Quaternion quaternion() const { return q_; }

    Vector3 euler_rpy_rad() const 
    {
        const Scalar sinr_cosp = Scalar(2) * (q_.w() * q_.x() + q_.y() * q_.z());
        const Scalar cosr_cosp = Scalar(1) - Scalar(2) * (q_.x() * q_.x() + q_.y() * q_.y());
        const Scalar roll = std::atan2(sinr_cosp, cosr_cosp);

        const Scalar sinp = Scalar(2) * (q_.w() * q_.y() - q_.z() * q_.x());
        const Scalar pitch = std::abs(sinp) >= Scalar(1) ? std::copysign(pi() / Scalar(2), sinp) : std::asin(sinp);

        const Scalar siny_cosp = Scalar(2) * (q_.w() * q_.z() + q_.x() * q_.y());
        const Scalar cosy_cosp = Scalar(1) - Scalar(2) * (q_.y() * q_.y() + q_.z() * q_.z());
        const Scalar yaw = std::atan2(siny_cosp, cosy_cosp);

        return Vector3(roll, pitch, yaw);
    }

    /**
     * @brief 世界天向在机体系里的分量，单位向量（倾角判断/重力补偿用）
     */
    Vector3 up_body() const { return q_.inverse() * world_up(); }

private:
    // 世界天向：gz/SDF 的世界是 Z 朝上，天向就是 +Z（重力是 -9.8 沿 Z，别搞反）
    static Vector3 world_up()  { return Vector3(Scalar(0), Scalar(0), Scalar(1)); }
    static constexpr Scalar pi() { return Scalar(3.141592653589793238462643383279502884L); }

    Quaternion q_{Quaternion::Identity()};
    Scalar accel_correction_gain_{Scalar(0.1)};
};

} // namespace algorithm::filter
