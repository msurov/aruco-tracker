#include "pnp_4pts.h"


cv::Matx<double,8,6> pnp_jac(
    cv::Matx33d const& K,
    cv::Vec3d const& r,
    cv::Vec3d const& t,
    cv::Matx<double,4,3> const& objpts
    )
{
    cv::Matx33d R = rodrigues_to_rotmat(r);
    cv::Matx<double,8,6> Jac;

    for (int i = 0; i < 4; ++ i)
    {
        auto q = row_as_vec(objpts, i);
        auto m = K * (R * q + t);
        double mx = m(0), my = m(1), mz = m(2);
        double mz2 = square(mz);
        cv::Matx23d A = cv::Matx23d(
            1/mz, 0, -mx/mz2,
            0, 1/mz, -my/mz2
        );
        cv::Matx23d Jr = -A * K * R * wedge(q);
        cv::Matx23d Jt = A * K;
        copy(Jac, Jr, i*2, 0);
        copy(Jac, Jt, i*2, 3);
    }

    return Jac;
}

bool solve_pnp_4pts(
    cv::Matx<double,4,3> const& objpts,
    cv::Matx<double,4,2> const& impts,
    cv::Matx<double,8,8> const& cov_impts,
    CameraIntrinsics const& intr,
    cv::Vec3d& r,
    cv::Vec3d& t,
    cv::Matx66d& cov_rp
)
{
    cv::Matx<float,4,3> objpts_ = objpts;
    cv::Matx<float,4,2> impts_ = impts;

    bool ans = cv::solvePnP(
        objpts_,
        impts_,
        intr.K,
        intr.distortion,
        r,
        t,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    if (!ans)
        return false;

    // cov_rp
    cv::Matx<double,8,6> Jac = pnp_jac(intr.K, r, t, objpts);
    cv::Matx<double,6,8> Jac_pi;
    cv::invert(Jac, Jac_pi, cv::DECOMP_SVD);
    cov_rp = Jac_pi * cov_impts * Jac_pi.t();

    return true;
}


double reprojection_error(
    cv::Matx<double,4,3> const& objpts,
    cv::Matx<double,4,2> const& impts,
    CameraIntrinsics const& intr,
    cv::Vec3d const& r,
    cv::Vec3d const& t)
{
    std::vector<cv::Vec2d> projected(4);
    cv::projectPoints(objpts, r, t, intr.K, intr.distortion, projected);
    double dv2_max = 0.;

    for (int i = 0; i < 4; ++ i)
    {
        cv::Vec2d v1 {impts(i,0), impts(i,1)};
        cv::Vec2d const& v2 = projected[i];
        auto dv = v2 - v1;
        dv2_max = std::max(dv2_max, (dv).dot(dv));
    }
    return std::sqrt(dv2_max);
}


cv::Vec<double,6> pose_error(
    cv::Matx<double,4,3> const& objpts,
    cv::Matx<double,4,2> const& impts,
    CameraIntrinsics const& intr,
    cv::Vec3d const& r,
    cv::Vec3d const& t
)
{
    std::vector<cv::Vec2d> projected(4);
    cv::projectPoints(objpts, r, t, intr.K, intr.distortion, projected);
    cv::Vec<double,8> delta_u;

    for (int i = 0; i < 4; ++ i)
    {
        delta_u(2*i) = projected[i](0) - impts(i,0);
        delta_u(2*i + 1) = projected[i](1) - impts(i,1);
    }

    cv::Matx<double,8,6> Jac = pnp_jac(intr.K, r, t, objpts);
    cv::Matx<double,6,8> Jac_pi;
    cv::invert(Jac, Jac_pi, cv::DECOMP_SVD);
    cv::Vec<double,6> delta_rt = Jac_pi * delta_u;
    return delta_rt;
}
