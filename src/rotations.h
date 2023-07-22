#pragma once
#include <ostream>
#include <math.h>
#include "quat.h"

template <typename T>
struct AngleAxisT
{
    T angle;
    cv::Vec<T,3> axis;

    AngleAxisT() : angle(0), axis({1,0,0}) {}

    AngleAxisT(T angle, cv::Vec<T,3> const& axis)
    {
        T n = norm(axis);
        if (n < 1e-12)
        {
            this->angle = 0.;
            this->axis = {1.,0.,0.};
        }
        else
        {
            this->angle = angle;
            this->axis = axis / n;
        }
    }
};

using AngleAxis = AngleAxisT<double>;

/**
 * @param theta angle
 * @param l axis
 * @result rotation matrix
 */
template <typename T>
inline cv::Matx<T, 3, 3> rotmat(T theta, cv::Vec<T, 3> const& l)
{
    const auto cos_theta = std::cos(theta);
    const auto sin_theta = std::sin(theta);
    return
        cos_theta * cv::Matx<T, 3, 3>::eye() + 
        sin_theta * wedge(l) + 
        outer(l, l) * (1 - cos_theta);
}

template <typename T>
inline cv::Matx<T, 3, 3> rotmat(AngleAxisT<T> const& aa)
{
    return rotmat(aa.angle, aa.axis);
}

/**
 * @param q quaternion
 * @result rotation matrix
 */
template <typename T>
inline cv::Matx<T,3,3> rotmat(QuatT<T> const& q)
{
    QuatT<T> q_ = normalized(q);
    T
        w = q_.w(),
        x = q_.x(),
        y = q_.y(),
        z = q_.z();
    T
        x2 = square(x),
        y2 = square(y),
        z2 = square(z);

    return {
          1 - 2 * y2 - 2 * z2,  2 * x * y - 2 * z * w,  2 * x * z + 2 * y * w,
        2 * x * y + 2 * z * w,    1 - 2 * x2 - 2 * z2,  2 * y * z - 2 * x * w,
        2 * x * z - 2 * y * w,  2 * y * z + 2 * x * w,    1 - 2 * x2 - 2 * y2
    };
}

/**
 * @param q quaternion
 * @result rodrigues
 */
template <typename T>
inline cv::Vec<T,3> rodrigues(QuatT<T> const& q)
{
    if (1 - std::abs(q.w()) <= 0.)
        return {0., 0., 0.};
    T theta = 2 * std::acos(q.w());
    return theta * q.vec() / sin(theta / 2.);
}

/**
 * @param r rodrigue parameters
 * @result quaternion
 */
template <typename T>
inline QuatT<T> quat(cv::Vec<T,3> const& rodrigues)
{
    const auto theta = norm(rodrigues);
    if (std::abs(theta) < 1e-15)
        return {1., 0., 0., 0.};
    const auto l = rodrigues / theta;
    return {std::cos(theta / 2), l * std::sin(theta / 2)};
}

template <typename T>
inline cv::Vec<T,3> rotate(QuatT<T> const& q, cv::Vec<T,3> const& v)
{
    return (q * v * q.conj()).vec();
}

template <typename T>
inline cv::Vec<T,3> rotate(cv::Vec<T,3> const& rodrigues, cv::Vec<T,3> const& v)
{
    const auto q = quat(rodrigues);
    return (q * v * q.conj()).vec();
}

/**
 * @param R rotation matrix to quaternion
 */
template <typename T>
inline QuatT<T> quat(cv::Matx<T,3,3> const& R)
{
    auto t = cv::trace(R);
    auto r = std::sqrt(1. + t);
    auto w = r / 2.;
    auto x = std::sqrt(std::abs(1 + R(0, 0) - R(1, 1) - R(2, 2))) / 2;

    if (R(2, 1) < R(1, 2))
        x = -x;
    auto y = std::sqrt(std::abs(1 - R(0, 0) + R(1, 1) - R(2, 2))) / 2;
    if (R(0, 2) < R(2, 0))
        y = -y;
    auto z = std::sqrt(std::abs(1 - R(0, 0) - R(1, 1) + R(2, 2))) / 2;
    if (R(1, 0) < R(0, 1))
        z = -z;

    return {w,x,y,z};
}

/**
 * @brief angleaxis to quaternion
 **/
template <typename T>
inline QuatT<T> quat(AngleAxisT<T> const& aa)
{
    const auto l = normalized(aa.axis);
    const auto theta = aa.angle;
    return QuatT<T>(std::cos(theta / 2.), l * std::sin(theta / 2.));
}

/**
 * @brief quaternion to angle-axis
 **/
template <typename T>
inline AngleAxisT<T> angleaxis(QuatT<T> const& q)
{
    if (1 - std::abs(q.w()) <= 0.)
        return {0., {1., 0., 0.}};
    auto theta = 2 * std::acos(q.w());
    auto l = q.vec() / sin(theta / 2.);
    return {theta, l};
}

/**
 * @brief rotation matrix to angle-axis
 **/
template <typename T>
inline AngleAxisT<T> angleaxis(cv::Matx<T,3,3> const& R)
{
    return angleaxis(quat(R));
}

/**
 * @brief convert rodrigue's to rotmat
 **/
template <typename T>
inline cv::Matx<T,3,3> rotmat(cv::Vec<T,3> const& rodrigues)
{
    auto theta = norm(rodrigues);
    if (theta < 1e-12)
        return cv::Matx<T,3,3>::eye();
    return rotmat(theta, rodrigues / theta);
}

/**
 * @brief convert rotmat to rodrigue's
 **/
template <typename T>
inline cv::Vec<T,3> rodrigues(cv::Matx<T,3,3> const& R)
{
    const auto v = vee(skew_part(R));
    const auto nv = norm(v);
    if (nv < 1e-12)
        return cv::Vec<T,3>::zeros();
    const auto theta = std::acos((cv::trace(R) - 1) / 2);
    return v / nv * theta;
}

/**
 * @brief angleaxis to stdout
 **/
template <typename T>
inline std::ostream& operator << (std::ostream& s, AngleAxisT<T> const& aa)
{
    char buf[256];
    int n = snprintf(
        buf, sizeof(buf), "{%f, (%f, %f, %f)}", 
        aa.angle, aa.axis(0), aa.axis(1), aa.axis(2));
    if (n > 0)
        s << buf;
    return s;
}

