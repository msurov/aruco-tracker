#pragma once

#include <opencv2/core.hpp>


template <typename T>
inline T square(T v)
{
    return v * v;
}

template <typename T>
inline T cube(T v)
{
    return v * v * v;
}

template <typename T>
inline T sign(T const& a)
{
    return a < T(0) ? T(-1) :
        a > 0 ? T(1) : T(0);
}

template <typename T>
inline T ramp(T const& a)
{
    return a < T(0) ? T(0) : a;
}

template <typename T>
inline cv::Matx<T,3,3> wedge(cv::Vec<T,3> const& a)
{
    auto x = a(0), y = a(1), z = a(2);
    return {
         0, -z,  y,
         z,  0, -x,
        -y,  x,  0
    };
}

template <typename T>
inline cv::Vec<T, 3> vee(cv::Matx<T, 3, 3> const& A)
{
    return {A(2,1), A(0, 2), A(1, 0)};
}

template <typename T, int n>
inline cv::Matx<T, n, n> skew_part(cv::Matx<T, n, n> const& A)
{
    return T(0.5) * (A - A.t());
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

template <typename T, int d, int nargs> 
inline cv::Matx<T, d * nargs, d * nargs> 
compose_diag(const cv::Matx<T, d, d>(&blocks)[nargs])
{
    auto dst = cv::Matx<T, d * nargs, d * nargs>::zeros();
    for (int i = 0; i < nargs; ++ i)
        copy(dst, blocks[i], i * d, i * d);
    return dst;
}

template <typename T, int n, int m, int nrows, int ncols> 
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


template <typename T>
inline T cross(cv::Vec<T,2> const& a, cv::Vec<T,2> const& b)
{
    return a(0) * b(1) - a(1) * b(0);
}

template <typename T>
inline cv::Vec<T,3> cross(cv::Vec<T,3> const& a, cv::Vec<T,3> const& b)
{
    return {
        a(1) * b(2) - a(2) * b(1),
        a(2) * b(0) - a(0) * b(2),
        a(0) * b(1) - a(1) * b(0)
    };
}

template <typename T, int N>
inline T dot(cv::Vec<T,N> const& a, cv::Vec<T,N> const& b)
{
    return (a.t() * b)(0);
}

template <typename T, int N>
inline T norm_squared(cv::Vec<T,N> const& a)
{
    return dot(a, a);
}

template <typename T, int N>
inline T norm(cv::Vec<T,N> const& a)
{
    return std::sqrt(norm_squared(a));
}

template <typename T, int N>
inline cv::Vec<T,N> normalized(cv::Vec<T,N> const& v)
{
    auto nv = norm(v);
    return v / std::max(nv, 1e-12);
}

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

inline cv::Matx33d wedge(cv::Vec3d const& a)
{
    double x = a(0), y = a(1), z = a(2);
    return cv::Matx33d(
         0, -z,  y,
         z,  0, -x,
        -y,  x,  0
    );
}
