#pragma once

#include <opencv2/opencv.hpp>
#include <math.h>
#include <tuple>


template <typename T>
inline T sign(T const& a)
{
    return a > static_cast<T>(0) ? static_cast<T>(1) : static_cast<T>(-1);
}

inline cv::Matx33f get_closest_rotmat(cv::Matx33f const& M)
{
    cv::Matx33f u, vt;
    cv::Vec3f w;
    cv::SVDecomp(M, w, u, vt);
    cv::Matx33f R = u * vt;
    return R;
}

inline cv::Matx33f quat_to_rotmat(cv::Vec4f const& q)
{
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    cv::Matx33f R(
        1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,       2*x*z + 2*y*w,
            2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,       2*y*z - 2*x*w,
            2*x*z - 2*y*w,     2*y*z + 2*x*w,   1 - 2*x*x - 2*y*y
    );

    return R;
}

inline cv::Vec4f rotmat_to_quat(cv::Matx33f const& R)
{
    float t = cv::trace(R);
    float r = sqrtf(1.f + t);
    float s = 0.5f / r;
    float w = 0.5f * r;
    float x = (R(2,1) - R(1,2)) * s;
    float y = (R(0,2) - R(2,0)) * s;
    float z = (R(1,0) - R(0,1)) * s;
    cv::Vec4f quat;
    return cv::Vec4f(w,x,y,z);
}

inline std::tuple<float, cv::Vec3f> quat_to_angle_axis(cv::Vec4f const& q)
{
    float theta = 2 * acosf(q[0]);
    if (fabsf(theta) < 1e-14f)
        return std::make_tuple(0.f, cv::Vec3f(0,0,0));

    float s = sinf(acosf(q[0]));
    cv::Vec3f l(q[1] / s, q[2] / s, q[3] / s);
    return std::make_tuple(theta, l);
}

inline cv::Vec3f quat_to_rodrigues(cv::Vec4f const& q)
{
    float theta;
    cv::Vec3f l;
    std::tie(theta,l) = quat_to_angle_axis(q);
    return l * theta;
}

inline cv::Vec4f quat_from_angle_axis(float angle, cv::Vec3f const& axis)
{
    if (fabs(angle) < 1e-14f)
    {
        return cv::Vec4f(1., 0., 0., 0.);
    }

    cv::Vec3f l = axis / sqrtf(axis.dot(axis));
    float c = cosf(angle/2);
    float s = sinf(angle/2);

    return cv::Vec4f(c, s * l[0], s * l[1], s * l[2]);
}

inline cv::Vec4f rodrigues_to_quat(cv::Vec3f const& r)
{
    float r_norm = sqrtf(r.dot(r));
    float theta = r_norm;
    cv::Vec3f l = r / r_norm;
    return quat_from_angle_axis(theta, l);
}

inline cv::Vec4f rodrigues_to_quat_v2(cv::Vec3f const& r)
{
    cv::Matx33f R;
    cv::Rodrigues(r, R);
    return rotmat_to_quat(R);
}

inline cv::Matx33f rodrigues_to_rotmat(cv::Vec3f const& r)
{
    cv::Matx33f R;
    cv::Rodrigues(r, R);
    return R;
}

template <int n>
inline cv::Vec<float,n> get_col(cv::Matx<float,n,n> const& m, int c)
{
    cv::Vec<float,n> col;

    for (int i = 0; i < n; ++ i)
        col[i] = m(i, c);

    return col;
}

inline std::tuple<cv::Vec3f, cv::Vec4f> decompose_homogeneous_mat(cv::Matx44f const& T)
{
    cv::Vec3f d(T(0, 3), T(1, 3), T(2, 3));
    cv::Matx33f R(
        T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2),
        T(2,0), T(2,1), T(2,2)
    );
    cv::Vec4f q = rotmat_to_quat(R);
    return std::make_tuple(d, q);
}
