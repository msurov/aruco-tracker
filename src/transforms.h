#pragma once

#include <cstring>
#include <assert.h>
#include "rotations.h"


struct Pose
{
    cv::Vec3d r; // rotation (in rodrigues)
    cv::Vec3d p; // displacement (in meters)
};

struct PoseCov : Pose
{
    cv::Matx<double,6,6> cov; // covariance matrix of (r,p)
};

inline Pose inverse(Pose const& pose)
{
    const cv::Vec3d r_new = -pose.r;
    const Quat q_conj = quat(r_new);
    const cv::Vec3d p_new = rotate(q_conj, -pose.p);
    return {p_new, r_new};
}

inline Pose compose(Pose const& pose_ab, Pose const& pose_bc)
{
    const Quat q_ab = quat(pose_ab.r);
    const Quat q_ac = q_ab * quat(pose_bc.r);
    const cv::Vec3d r_ac = rodrigues(q_ac);
    const cv::Vec3d& p_ab = pose_ab.p;
    const cv::Vec3d& p_bc = pose_bc.p;
    const cv::Vec3d p_ac = rotate(q_ab, p_bc) + p_ab;
    return {p_ac, r_ac};
}

inline cv::Vec3d transform(Pose const& pose_ab, cv::Vec3d const& point_b)
{
    return rotate(pose_ab.r, point_b) + pose_ab.p;
}
