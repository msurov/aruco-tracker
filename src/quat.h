#pragma once

#include <ostream>
#include "cvmath.h"


template <typename T=double>
struct QuatT : public cv::Vec<T, 4>
{
private:
    using inheritance = cv::Vec<T,4>;
    using Vec = cv::Vec<T,3>;

public:
    QuatT() {}
    QuatT(T w, T x, T y, T z) : inheritance({w,x,y,z}) {}
    QuatT(T w) : QuatT(w, 0, 0, 0) {}
    QuatT(T w, Vec const& v) : QuatT(w, v(0), v(1), v(2)) {}
    QuatT(inheritance const& elems) : inheritance(elems) {}

    inline QuatT mul(QuatT const& q) const
    {
        return QuatT(
            w() * q.w() - dot(vec(), q.vec()),
            w() * q.vec() + vec() * q.w() + cross(vec(), q.vec())
        );
    }
    inline QuatT mul(Vec const& qv) const
    {
        return QuatT(
            -dot(vec(), qv),
            w() * qv + cross(vec(), qv)
        );
    }
    inline T w() const { return (*this)(0); }
    inline T x() const { return (*this)(1); }
    inline T y() const { return (*this)(2); }
    inline T z() const { return (*this)(3); }
    inline T& w() { return (*this)(0); }
    inline T& x() { return (*this)(1); }
    inline T& y() { return (*this)(2); }
    inline T& z() { return (*this)(3); }
    Vec vec() const { return { x(), y(), z() }; }
    inline QuatT conj() const { return QuatT(w(), -vec()); }
};

template <class T>
inline QuatT<T> operator * (QuatT<T> const& a, QuatT<T> const& b)
{
    return a.mul(b);
}

template <class T>
inline QuatT<T> operator * (QuatT<T> const& a, cv::Vec<T,3> const& b)
{
    return a.mul(b);
}


using Quat = QuatT<>;

inline std::ostream& operator << (std::ostream& s, Quat const& q)
{
    char buf[4*33];
    sprintf(buf, "(%f; %f, %f, %f)", q.w(), q.x(), q.y(), q.z());
    s << buf;
    return s;
}

inline double quat_rot_proj(Quat const& q, cv::Vec3d const& l_)
{
    auto l = normalized(l_);
    double w = q.w();
    cv::Vec3d v = q.vec();
    return 2 * atan2(dot(v, l), w);
}
