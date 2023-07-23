#include <cppmisc/json.h>
#include "transforms.h"


template <typename T, int N, int M>
Json::Value to_json(cv::Matx<T, N, M> const& mat)
{
    Json::Value result;

    if constexpr (N == 1)
    {
        for (int i = 0; i < M; ++ i)
            result.append(mat(0, i));
    }
    else if constexpr (M == 1)
    {
        for (int i = 0; i < N; ++ i)
            result.append(mat(i, 0));
    }
    else
    {
        for (int i = 0; i < N; ++ i)
        {
            Json::Value row;
            for (int j = 0; j < M; ++ j)
            {
                row.append(mat(i, j));
            }
            result.append(row);
        }
    }

    return result;
}

Json::Value to_json(Pose const& pose)
{
    Json::Value result;
    result["position"] = to_json(pose.p);
    result["orientation"] = to_json(pose.r);
    return result;
}
