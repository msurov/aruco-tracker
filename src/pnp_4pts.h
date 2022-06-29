#pragma once
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <cppmath/math.h>
#include "cvmath.h"


/**
 * @brief \f[ \frac{\partial u}{\partial\left(r,t\right)} \f]
 *  partial derivative of pixel coordinates of projection point u wrt object pose (r,t)
 * @param K camera matrix
 * @param r rodrigues, object orientation 
 * @param t object position
 * @param objpts object points wrt object-related frame
 */
cv::Matx<double,8,6> pnp_jac(
    cv::Matx33d const& K,
    cv::Vec3d const& r,
    cv::Vec3d const& t,
    cv::Matx<double,4,3> const& objpts
);

bool solve_pnp_4pts(
    cv::Matx<double,4,3> const& objpts,
    cv::Matx<double,4,2> const& impts,
    cv::Matx<double,8,8> const& cov_impts,
    CameraIntrinsics const& intr,
    cv::Vec3d& r,
    cv::Vec3d& t,
    cv::Matx66d& cov_rp
);


/**
 * @brief maximum deviation of expected 
 *  projection coordinates and real
 */
double reprojection_error(
    cv::Matx<double,4,3> const& objpts,
    cv::Matx<double,4,2> const& impts,
    CameraIntrinsics const& intr,
    cv::Vec3d const& r,
    cv::Vec3d const& t
);


/**
 * @brief delta (r,t) for the given pnp solution
 */
cv::Vec<double,6> pose_error(
    cv::Matx<double,4,3> const& objpts,
    cv::Matx<double,4,2> const& impts,
    CameraIntrinsics const& intr,
    cv::Vec3d const& r,
    cv::Vec3d const& t
);
