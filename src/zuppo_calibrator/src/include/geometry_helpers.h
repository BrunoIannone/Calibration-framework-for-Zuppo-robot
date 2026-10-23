#pragma once

#include <Eigen/Geometry>
#include <cmath>

/** @brief Converts a 2D isometry to a 3D vector (x, y, theta)
 * @tparam Scalar Floating-point scalar type (e.g., `float`, `double`).
 * @param T The input 2D rigid transformation (Isometry).
 * @return Eigen::Matrix<Scalar, 3, 1> A 3-element vector containing [x, y, theta].
 */
template <typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> t2v(const Eigen::Transform<Scalar, 2, Eigen::Isometry> &T)
{
    Eigen::Matrix<Scalar, 3, 1> v;
    v.template head<2>() = T.translation();
    v(2) = std::atan2(T.linear()(1, 0), T.linear()(0, 0));
    return v;
}

/** @brief Converts a 3D vector (x, y, theta) to a 2D isometry transformation.
 * @tparam Scalar Floating-point scalar type (e.g., `float`, `double`).
 * @param v The input vector containing [x, y, theta].
 * @return An Eigen::Isometry2d representing the transformation.
 */

template <typename Scalar>
inline Eigen::Transform<Scalar, 2, Eigen::Isometry> v2t(const Eigen::Matrix<Scalar, 3, 1> &v)
{
    Eigen::Transform<Scalar, 2, Eigen::Isometry> T = Eigen::Transform<Scalar, 2, Eigen::Isometry>::Identity();
    T.translation() = v.template head<2>();
    T.linear() = Eigen::Rotation2D<Scalar>(v.z()).toRotationMatrix();
    return T;
}

/**
 * @brief Projects a 3D rigid pose (Isometry3f) down to a 2D planar pose (Isometry2f).
 * 
 * Extracts x and y translation, dropping z. Calculates planar yaw from the 3D rotation matrix.
 * 
 * @param iso The 3D input isometry.
 * @return Eigen::Isometry2f The projected 2D rigid transformation.
 */
inline Eigen::Isometry2f iso3dTo2d(const Eigen::Isometry3f &iso)
{
    Eigen::Isometry2f iso2d = Eigen::Isometry2f::Identity();
    iso2d.translation() = iso.translation().head<2>();

    const float yaw = std::atan2(iso.linear()(1, 0), iso.linear()(0, 0));
    iso2d.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();

    return iso2d;
}

/**
 * @brief Embeds a 2D planar pose (Isometry2f) into a 3D rigid pose (Isometry3f).
 * 
 * Sets z-translation to 0.
 * 
 * @param iso The 2D input isometry.
 * @return Eigen::Isometry3f The extended 3D rigid transformation.
 */
inline Eigen::Isometry3f iso2dTo3d(const Eigen::Isometry2f &iso)
{

    Eigen::Isometry3f iso3d = Eigen::Isometry3f::Identity();
    iso3d.linear().topLeftCorner<2, 2>() = iso.linear();

    iso3d.translation().head<2>() = iso.translation();
    iso3d.translation().z() = 0;

    return iso3d;
}

// 3D HELPERS

/**
 * @brief Computes a 3D rotation matrix from Euler angles using standard intrinsic X-Y-Z order.
 * 
 * @param a A 3-element vector containing Euler angles [roll (X), pitch (Y), yaw (Z)] in radians.
 * @return Eigen::Matrix3f The resulting rotation matrix.
 */
inline Eigen::Matrix3f angles2R(const Eigen::Vector3f &a)
{
    // a[0] = roll/X, a[1] = pitch/Y, a[2] = yaw/Z
    Eigen::AngleAxisf rotX(a[0], Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotY(a[1], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotZ(a[2], Eigen::Vector3f::UnitZ());

    return (rotX * rotY * rotZ).toRotationMatrix();
}

/**
 * @brief Converts a 6D vector (x, y, z, roll, pitch, yaw) to a 4x4 homogenous transformation matrix.
 * 
 * @param v A 6-element vector containing translation [0..2] and Euler angles [3..5].
 * @return Eigen::Matrix4f The 4x4 homogenous transformation matrix.
 */
inline Eigen::Matrix4f v2t(const Eigen::Matrix<float, 6, 1> &v)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = angles2R(v.tail<3>());
    T.block<3, 1>(0, 3) = v.head<3>();

    return T;
}

/**
 * @brief Extracts Euler angles [roll, pitch, yaw] from a 3x3 rotation matrix using X-Y-Z sequence.
 * 
 * Includes protection against gimbal lock singularities (when pitch approaches +/- 90 degrees).
 * 
 * @param R The 3x3 rotation matrix.
 * @return Eigen::Vector3f Vector containing Euler angles [roll, pitch, yaw] in radians.
 */
inline Eigen::Vector3f R2Angles(const Eigen::Matrix3f &R)
{
    Eigen::Vector3f angles = Eigen::Vector3f::Zero();

    const float cos_beta = std::sqrt(R(0, 0) * R(0, 0) + R(0, 1) * R(0, 1));

    if (cos_beta > 1e-6)
    {
        // Normal case
        angles[0] = std::atan2(-R(1, 2), R(2, 2)); // Roll  (alpha)
        angles[1] = std::atan2(R(0, 2), cos_beta); // Pitch (beta)
        angles[2] = std::atan2(-R(0, 1), R(0, 0)); // Yaw   (gamma)
    }
    else
    {
        // Gimbal lock case (Pitch is +/- 90 degrees)
        angles[1] = std::atan2(R(0, 2), cos_beta);
        angles[0] = std::atan2(R(2, 1), R(1, 1)); // Combined Roll/Yaw
        angles[2] = 0.0;                          // Arbitrary assignment
    }

    return angles;
}

/**
 * @brief Extracts the scalar rotation angle (yaw) in radians from a 2D isometry.
 * 
 * @param T The 2D rigid transformation matrix.
 * @return float The rotation angle in radians within the range [-pi, pi].
 */
inline float getYaw(const Eigen::Isometry2f &T)
{
    Eigen::Rotation2Df R(T.rotation());

    return R.angle();
}

/**
 * @brief Converts a 4x4 homogenous transformation matrix into a 6D pose vector [x, y, z, roll, pitch, yaw].
 * 
 * @param T The 4x4 homogenous transformation matrix.
 * @return Eigen::Matrix<float, 6, 1> Vector representing 3D position and orientation.
 */
inline Eigen::Matrix<float, 6, 1> t2v(const Eigen::Matrix4f &T)
{
    Eigen::Matrix<float, 6, 1> v;

    v.head<3>() = T.block<3, 1>(0, 3);

    v.tail<3>() = R2Angles(T.block<3, 3>(0, 0));

    return v;
}
/// @brief Generic 3x1 column vector type alias.
template <typename Scalar_>
using Vector3 = Eigen::Matrix<Scalar_, 3, 1>;

/// @brief Generic 3x3 matrix type alias.
template <typename Scalar_>
using Matrix3 = Eigen::Matrix<Scalar_, 3, 3>;

/**
 * @brief Constructs a 3x3 skew-symmetric matrix from a 3D vector. 
 * @tparam Scalar_ Floating-point scalar type.
 * @param v The 3D input vector.
 * @return Matrix3<Scalar_> The 3x3 skew-symmetric matrix.
 */
template <typename Scalar_>
inline Matrix3<Scalar_> skew(const Vector3<Scalar_> v)
{
    Matrix3<Scalar_> S;
    const Scalar_ zero(0.);
    S << zero, -v[2], v[1],
        v[2], zero, -v[0],
        -v[1], v[0], zero;
    return S;
}