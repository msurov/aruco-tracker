#pragma once

#include "traces.h"
#include "math.h"


template <class Signal>
class DiffFilter
{
private:
    int64_t _t;
    Signal _x1, _x2;
    double _T;

    double a11, a12, a21, a22;
    double b1, b2;
    double c1, c2;

    inline void set_initial_state(int64_t t, Signal const& u)
    {
        Eigen::Matrix2d A;
        A << a11, a12,
            a21, a22;
        Eigen::Vector2d B;
        Eigen::Matrix<double, 1, 2> C;
        B << b1, b2;
        C << c1, c2;

        Eigen::Matrix2d P;
        Eigen::Vector2d CA = C * A;
        double CB = C * B;
        P << c1, c2,
            CA(0), CA(1);
        Eigen::Vector2d Q;
        Q << 0, -CB;
        Eigen::Vector2d k = P.inverse() * Q;
        _x1 = k(0) * u;
        _x2 = k(1) * u;
        _t = t;
    }

public:
    DiffFilter() = delete;
    DiffFilter(DiffFilter const&) = delete;
    DiffFilter(DiffFilter&&) = delete;

    DiffFilter(double T)
    {
        assert(T > 0);
        
        _T = T;
        _t = -1;

        a11 = -2 / _T;
        a12 = 1. / square(_T);
        a21 = -1.;
        a22 = 0.;

        b1 = 1. / square(_T);
        b2 = 0.;

        c1 = 1.;
        c2 = 0.;
    }

    inline void reset()
    {
        _t = -1;
    }

    Signal value() const
    {
        auto y = c1 * _x1 + c2 * _x2;
        return y;
    }

    Signal update(int64_t t, Signal const& u)
    {
        if (_t == -1)
        {
            set_initial_state(t, u);
        }
        else
        {
            double dt = (t - _t) * 1e-6;
            if (dt < 1e-14)
                return value();

            if (dt > _T / 2.)
                update(_t + dt / 2., u);

            auto dx1 = a11 * _x1 + a12 * _x2 + b1 * u;
            auto dx2 = a21 * _x1 + a22 * _x2 + b2 * u;
            _x1 += dx1 * dt;
            _x2 += dx2 * dt;
            _t = t;
        }

        return value();
    }
};
