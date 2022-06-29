#pragma once

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>


struct CameraIntrinsics
{
    cv::Matx33f K;
    std::vector<float> distortion;
    cv::Size2i resolution;
};

struct PoseCov
{
    cv::Vec3d r; // rotation (in rodrigues)
    cv::Vec3d p; // displacement (in meters)
    cv::Matx<double,6,6> cov; // covariance matrix of (r,p)
};

template <typename T, int n, int m>
static cv::Matx<T,n,m> outer(
    cv::Vec<T,n> const& a,
    cv::Vec<T,m> const& b
    )
{
    cv::Matx<T,n,m> ab;

    for (int y = 0; y < n; ++ y)
    {
        auto a_y = a[y];
        for (int x = 0; x < m; ++ x)
            ab(y,x) = a_y * b[x];
    }

    return ab;
}

template <typename T, int h1, int w1, int h2, int w2> 
static void copy(
    cv::Matx<T, h1, w1>& dst,
    cv::Matx<T, h2, w2> const& src,
    int y1,
    int x1
    )
{
    assert(h1 >= h2 + y1);
    assert(w1 >= w2 + x1);

    for (int y = 0; y < h2; ++ y)
    {
        for (int x = 0; x < w2; ++ x)
        {
            dst(y + y1, x + x1) = src(y, x);
        }
    }
}

template <typename T, int d, size_t nargs> 
inline cv::Matx<T, d * nargs, d * nargs> 
compose_diag(const cv::Matx<T, d, d>(&blocks)[nargs])
{
    auto dst = cv::Matx<T, d * nargs, d * nargs>::zeros();
    for (int i = 0; i < nargs; ++ i)
        copy(dst, blocks[i], i * d, i * d);
    return dst;
}

template <typename T, int n, int m, size_t nrows, size_t ncols> 
inline cv::Matx<T, n * nrows, m * ncols> 
compose_block(const cv::Matx<T, n, m>(&blocks)[nrows][ncols])
{
    cv::Matx<T, n * nrows, m * ncols> dst;

    for (int r = 0; r < nrows; ++ r)
    {
        for (int c = 0; c < ncols; ++c)
        {
            auto const& block = blocks[r][c];
            copy(dst, block, r * n, c * m);
        }
    }

    return dst;
}

template <typename T, int n, int k>
inline cv::Vec<T, n*k> concatenate(const cv::Vec<T, n>(&vecs)[k])
{
    cv::Vec<T, n*k> result;
    for (int i = 0; i < k; ++ i)
    {
        for (int j = 0; j < n; ++ j)
        {
            result(i*n + j) = vecs[i](j);
        }
    }
    return result;
}

template <typename T, int n>
inline cv::Vec<T,n> to_vec(cv::Matx<T,n,1> const& m)
{
    cv::Vec<T,n> v;
    for (int i = 0; i < n; ++ i)
        v(i) = m(i,0);
    return v;
}

template <typename T, int n>
inline cv::Vec<T,n> to_vec(cv::Matx<T,1,n> const& m)
{
    cv::Vec<T,n> v;
    for (int i = 0; i < n; ++ i)
        v(i) = m(0,i);
    return v;
}

template <typename T, int n, int m>
inline cv::Vec<T,m> row_as_vec(cv::Matx<T,n,m> const& mat, int row)
{
    cv::Vec<T,m> v;
    for (int i = 0; i < m; ++ i)
        v(i) = mat(row, i);
    return v;
}

inline cv::Point2i round(cv::Point2f const& p)
{
    return cv::Point2i(lroundf(p.x), lroundf(p.y));
}

inline cv::Point2i round(cv::Point2d const& p)
{
    return cv::Point2i(lround(p.x), lround(p.y));
}

template <typename T, int n>
inline cv::Vec<int,n> round(cv::Vec<T,n> const& v)
{
    cv::Vec<int,n> dst;
    for (int i = 0; i < n; ++ i)
        dst(i) = std::round(v(i));
    return dst;
}

inline cv::Matx33d rodrigues_to_rotmat(cv::Vec3d const& rodrigues)
{
    cv::Matx33d R;
    cv::Rodrigues(rodrigues, R);
    return R;
}

inline cv::Vec3d rotmat_to_rodrigues(cv::Matx33d const& R)
{
    cv::Vec3d rodrigues;
    cv::Rodrigues(R, rodrigues);
    return rodrigues;
}

inline cv::Matx33d wedge(cv::Vec3d const& a)
{
    double x = a(0), y = a(1), z = a(2);
    return cv::Matx33d(
         0, -z,  y,
         z,  0, -x,
        -y,  x,  0
    );
}
