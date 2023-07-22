#pragma once
#include <functional>


struct OptimizeResult
{
    int status;
    int nfev;
    bool success;
    double fun, x;
};

/**
 * @brief Find function minima in intreval [x1,x2]
 * @param func cost function
 * @param x1 interval lower bound
 * @param x2 interval upper bound
 * @param xatol absolute tolerance (stop condition)
 * @param maxiter maximum iterations
 * @result OptimizeResult
 */
OptimizeResult fminbound(
    std::function<double(double)> func,
    double x1, double x2,
    double xatol=1e-5, int maxiter=500
);
