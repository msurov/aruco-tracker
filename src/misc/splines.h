#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <misc/traces.h>


namespace Eigen
{

    // 
    // interpolate a set of points x,y 
    // by a vector valued B-spline y = y(x) of degree 3
    // with zero derivatives at ends, i.e.
    // dy/dx = 0 at x = x[0]
    // dy/dx = 0 at x = x[x.size() - 1]
    // 
    template <typename SplineType>
    struct SplineFitting2
    {
        typedef typename SplineType::KnotVectorType KnotVectorType;
        typedef typename SplineType::ParameterVectorType ParameterVectorType;
        typedef typename SplineType::ControlPointVectorType ControlPointVectorType;
        typedef typename SplineType::KnotVectorType::Scalar Scalar;
        typedef Matrix<Scalar,Dynamic,Dynamic> MatrixType;

        template <typename PointArrayType>
        static SplineType Interpolate(const KnotVectorType& x, const PointArrayType& y)
        {
            assert(y.rows() == x.size());
            assert(x.size() >= 2);

            const DenseIndex p = 3;
            const DenseIndex K = x.size();
            const DenseIndex N = K + p - 1;
            const DenseIndex T = K + 2 * p;

            KnotVectorType t = Eigen::VectorXd::Zero(T);
            t.segment(0,p).fill(x(0));
            t.segment(p,K) = x;
            t.segment(p+K,p).fill(x(K-1));
            MatrixType A = MatrixType::Zero(N,N);
            MatrixType b = MatrixType::Zero(N, y.cols());

            for (int k = 0; k < K; ++ k)
            {
                const DenseIndex l = SplineType::Span(x(k), p, t);
                auto const& B_arr = SplineType::BasisFunctions(x[k], p, t);
                A.row(k).segment(l-p, p+1) = B_arr;
                b.row(k) = y.row(k);
            }

            if (true)
            {
                const DenseIndex l = SplineType::Span(x(0), p, t);
                auto const& der = SplineType::BasisFunctionDerivatives(t(p), 1, p, t);
                A.row(K).segment(l-p, p+1) = der.row(1);
                // b.row(K) = der0;
            }

            if (true)
            {
                const DenseIndex l = SplineType::Span(x(K-1), p, t);
                auto const& der = SplineType::BasisFunctionDerivatives(t(K+p-1), 1, p, t);
                A.row(K+1).segment(l-p, p+1) = der.row(1);
                // b.row(K) = der1;
            }

            // dbg_msg("splines t:\n", t);
            // dbg_msg("splines A:\n", A);
            // dbg_msg("splines b:\n", b);

            Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
            Eigen::MatrixXd c = lu.solve(b);
            ControlPointVectorType _c = c.transpose();

            return SplineType(t, _c);
        }

        template <typename PointArrayType, typename DerivArrayType>
        static SplineType InterpolateWithBoundary(
            KnotVectorType const& x, 
            PointArrayType const& y,
            DerivArrayType const& dy
            )
        {
            assert(y.rows() == x.size());
            assert(dy.rows() == 2);
            assert(x.size() >= 2);

            const DenseIndex p = 3;
            const DenseIndex K = x.size();
            const DenseIndex N = K + p - 1;
            const DenseIndex T = K + 2 * p;

            KnotVectorType t = Eigen::VectorXd::Zero(T);
            t.segment(0,p).fill(x(0));
            t.segment(p,K) = x;
            t.segment(p+K,p).fill(x(K-1));
            MatrixType A = MatrixType::Zero(N,N);
            MatrixType b = MatrixType::Zero(N, y.cols());

            for (int k = 0; k < K; ++ k)
            {
                const DenseIndex l = SplineType::Span(x(k), p, t);
                auto const& B_arr = SplineType::BasisFunctions(x[k], p, t);
                A.row(k).segment(l-p, p+1) = B_arr;
                b.row(k) = y.row(k);
            }

            if (true)
            {
                const DenseIndex l = SplineType::Span(x(0), p, t);
                auto const& der = SplineType::BasisFunctionDerivatives(t(p), 1, p, t);
                A.row(K).segment(l-p, p+1) = der.row(1);
                b.row(K) = dy.row(0);
            }

            if (true)
            {
                const DenseIndex l = SplineType::Span(x(K-1), p, t);
                auto const& der = SplineType::BasisFunctionDerivatives(t(K+p-1), 1, p, t);
                A.row(K+1).segment(l-p, p+1) = der.row(1);
                b.row(K+1) = dy.row(1);
            }

            // dbg_msg("splines t:\n", t);
            // dbg_msg("splines A:\n", A);
            // dbg_msg("splines b:\n", b);

            Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
            Eigen::MatrixXd c = lu.solve(b);
            ControlPointVectorType _c = c.transpose();

            return SplineType(t, _c);
        }
    };
}
