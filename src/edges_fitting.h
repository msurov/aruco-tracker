#pragma once

#include <assert.h>
#include <opencv2/core.hpp>
#include "cvmath.h"


static
bool 
fit_line(
    cv::Mat const& weights,
    double thresh,
    double weights_deviation,
    cv::Vec3d& l,
    cv::Matx33d& cov_l
    )
{
    int const Ny = weights.rows;
    int const Nx = weights.cols;

    double W = 0.;
    double x0 = 0., y0 = 0.;

    for (int y = 0; y < Ny; ++ y)
    {
        uchar const* pw = weights.ptr<uchar>(y);

        for (int x = 0; x < Nx; ++ x)
        {
            double w = pw[x];
            if (w < thresh)
                continue;

            W += w;
            x0 += x * w;
            y0 += y * w;
        }
    }

    if (W < thresh * Nx)
        return false;

    double inv_W = 1 / W;
    x0 *= inv_W;
    y0 *= inv_W;

    cv::Matx22d A = cv::Matx22d::zeros();

    for (int y = 0; y < Ny; ++ y)
    {
        uchar const* pw = weights.ptr<uchar>(y);

        for (int x = 0; x < Nx; ++ x)
        {
            double w = pw[x];
            if (w < thresh)
                continue;

            double dx = x - x0;
            double dy = y - y0;

            A += cv::Matx22d(
                w*dx*dx, w*dx*dy,
                w*dy*dx, w*dy*dy
            );
        }
    }

    cv::Matx22d U,Vt;
    cv::Vec2d singular;
    cv::SVD::compute(A, singular, U, Vt);

    double vx = Vt(0,0),
        vy = Vt(0,1);
    cv::Vec2d v(vx, vy);
    cv::Vec2d v_perp(-vy, vx);
    auto lam = singular(0);
    cv::Vec3d k(
        vx, vy, -vx*x0 - vy*y0
    );
    k /= ((A * v_perp).dot(v_perp) - lam);

    l = cv::Vec3d(-vy, vx, vy * x0 - vx * y0);
    cov_l = cv::Matx33d::zeros();

    for (int y = 0; y < Ny; ++ y)
    {
        uchar const* pw = weights.ptr<uchar>(y);

        for (int x = 0; x < Nx; ++ x)
        {
            double w = pw[x];
            if (w < thresh)
                continue;

            cv::Vec2d d(x - x0, y - y0);
            cv::Vec3d delta_l = 
                v_perp.dot(d) * v.dot(d) * k + 
                cv::Vec3d(0, 0, -v_perp.dot(d)) * inv_W;
            cov_l += outer(delta_l, delta_l);
        }
    }

    cov_l *= square(weights_deviation);
    return true;
}

static cv::RotatedRect get_neib_rect(cv::Point2d const& p1, cv::Point2d const& p2, float sz)
{
    cv::Point2d center = (p1 + p2) * 0.5f;
    cv::Point2d v = p2 - p1;
    cv::Size2f rect_sz(sqrtf(v.dot(v)) - 1.f, sz);
    float alpha = atan2(v.y, v.x);
    return cv::RotatedRect(center, rect_sz, alpha);
}

inline cv::Matx33d make_translation(double dx, double dy)
{
    return cv::Matx33d(
        1, 0, dx,
        0, 1, dy,
        0, 0, 1
    );
}

inline cv::Matx33d make_rotation(double alpha)
{
    double c = cos(alpha);
    double s = sin(alpha);

    return cv::Matx33d(
         c, -s, 0.,
         s,  c, 0.,
        0., 0., 1.
    );
}

inline cv::Matx23d rigid_transform(cv::Matx33d const& T)
{
    return cv::Matx23d(
        T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2)
    );
}

void clear_outliers(cv::Mat& weights, cv::Vec3d const& l, double thresh)
{
    assert(weights.type() == CV_8U);
    const int Nx = weights.cols;
    const int Ny = weights.rows;

    double a = l(0);
    double b = l(1);
    double c = l(2);
    double n = sqrt(a*a + b*b);
    double an = a / n;
    double bn = b / n;
    double cn = c / n;

    for (int y = 0; y < Ny; ++ y)
    {
        uchar* p = weights.ptr(y);

        for (int x = 0; x < Nx; ++ x)
        {
            double d = fabs(an * x + bn * y + cn);
            if (d > thresh)
                p[x] = 0;
        }
    }
}

bool fit_edge(
    cv::Mat const& im,
    cv::Point2d const& p1,
    cv::Point2d const& p2,
    int brightness_threshold,
    double noise,
    int neighborhood_sz,
    cv::Vec3d& line,
    cv::Matx33d& cov_line
    )
{
    auto const& r = get_neib_rect(p1, p2, neighborhood_sz);
    cv::Matx33d T = 
        make_translation(r.size.width / 2.f, r.size.height / 2.f) * 
        make_rotation(-r.angle + M_PI) * 
        make_translation(-r.center.x, -r.center.y);
    cv::Matx23f _T = rigid_transform(T);
    cv::Mat cropped, grad;

    cv::warpAffine(im, cropped, _T, r.size, cv::INTER_LINEAR);
    cv::GaussianBlur(cropped, cropped, cv::Size(3,3), 1., 1., cv::BORDER_REPLICATE);
    cv::Sobel(cropped, grad, CV_8U, 0, 1, 3, 0.25, 0, cv::BORDER_DEFAULT);

    cv::Vec3d l1;
    cv::Matx33d cov_l1;
    bool b = fit_line(grad, brightness_threshold, noise, l1, cov_l1);
    if (!b)
        return false;

    clear_outliers(grad, l1, 2.);
    b = fit_line(grad, brightness_threshold, noise, l1, cov_l1);
    if (!b)
        return false;

    line = T.t() * l1;
    cov_line = T.t() * cov_l1 * T;
    return true;
}

/*
 * l1 = (a1,b1,c1) defines a line a1 * x + b1 * y + c1 = 0
 * l2 = (a2,b2,c2) defines a line a2 * x + b2 * y + c2 = 0
 * p is point of intersection of the lines
 * J1 is jacobian matrix d p / d l1
 * J2 is jacobian matrix d p / d l2
 */
void lines_crossing_point(
    cv::Vec3d const& l1,
    cv::Vec3d const& l2,
    cv::Vec2d& p,
    cv::Matx23d& J1,
    cv::Matx23d& J2
    )
{
    double
        a1 = l1(0), b1 = l1(1), c1 = l1(2),
        a2 = l2(0), b2 = l2(1), c2 = l2(2);

    cv::Matx22d A(
        a1, b1,
        a2, b2
    );
    cv::Vec2d B(-c1, -c2);
    auto sol = A.solve(B);
    double x = sol(0), y = sol(1);
    p = cv::Vec2d(x, y);

    cv::Matx23d D1(
        -x, -y, -1,
         0,  0,  0
    );
    cv::Matx23d D2(
        0, 0, 0,
        -x, -y, -1
    );
    J1 = A.solve(D1);
    J2 = A.solve(D2);
}

bool refine_quad(
    cv::Mat const& im,
    cv::Matx<double, 4, 2> const& quad,
    int threshold,
    cv::Matx<double, 4, 2>& refined,
    cv::Matx<double, 8, 8>& refined_cov
    )
{
    cv::Vec3d l1,l2,l3,l4;
    cv::Matx33d cov_l1, cov_l2, cov_l3, cov_l4; 

    cv::Vec2d
        p1(quad(0,0), quad(0,1)),
        p2(quad(1,0), quad(1,1)),
        p3(quad(2,0), quad(2,1)),
        p4(quad(3,0), quad(3,1));

    bool b;
    b = fit_edge(im, p1, p2, threshold, 3, 18, l1, cov_l1);
    if (!b)
        return false;
    b = fit_edge(im, p2, p3, threshold, 3, 18, l2, cov_l2);
    if (!b)
        return false;
    b = fit_edge(im, p3, p4, threshold, 3, 18, l3, cov_l3);
    if (!b)
        return false;
    b = fit_edge(im, p4, p1, threshold, 3, 18, l4, cov_l4);
    if (!b)
        return false;

    cv::Matx23d J14, J11, J21, J22, J32, J33, J43, J44;
    
    lines_crossing_point(l4, l1, p1, J14, J11);
    lines_crossing_point(l1, l2, p2, J21, J22);
    lines_crossing_point(l2, l3, p3, J32, J33);
    lines_crossing_point(l3, l4, p4, J43, J44);

    auto Cov_l = compose_diag({cov_l1, cov_l2, cov_l3, cov_l4});
    auto Z = cv::Matx23d::zeros();
    auto Jac_p = compose_block({
        {J11, Z, Z, J14},
        {J21, J22, Z, Z},
        {Z, J32, J33, Z},
        {Z, Z, J43, J44}
    });
    auto Cov_p = Jac_p * Cov_l * Jac_p.t();
    refined_cov = Cov_p;
    refined = compose_block({
        {p1.t()},
        {p2.t()},
        {p3.t()},
        {p4.t()}
    });
    return true;
}
