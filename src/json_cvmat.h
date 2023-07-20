#pragma once

#include <cppmisc/json.h>
#include "cvmath.h"

template <typename T, int N>
struct Converter<cv::Vec<T, N>>
{
    using DType = cv::Vec<T, N>;

    static DType doit(Json::Value const& json)
    {
        DType vec;
        if (N != json.size())
            throw_exception<Json::ConvertationError>(
                "Expect vector of ", N, " elements, but ", json.size(), " presented");

        for (int i = 0; i < N; ++ i)
            vec(i) = Converter<T>::doit(json[i]);

        return vec;
    }
};

template <typename T, int Rows, int Cols>
struct Converter<cv::Matx<T, Rows, Cols>>
{
    using DType = cv::Matx<T, Rows, Cols>;

    static DType doit(Json::Value const& json)
    {
        DType mat;

        if constexpr (Rows == 1 || Cols == 1)
        {
            constexpr int n = std::max(Rows, Cols);
            mat = Converter<cv::Vec<T, n>>::doit(json);
        }
        else
        {
            if (Rows != json.size())
                throw_exception<Json::ConvertationError>(
                    "Expect matrix of ", Rows, " rows, but ", json.size(), " presented");

            for (int i = 0; i < Rows; ++ i)
            {
                Json::Value jsonrow = json[i];

                if (Cols != json.size())
                    throw_exception<Json::ConvertationError>(
                        "Expect matrix of ", Cols, " columns, but ", jsonrow.size(), " presented");

                for (int j = 0; j < Cols; ++ j)
                    mat(i,j) = Converter<T>::doit(jsonrow[j]);
            }
        }

        return mat;
    }
};

template <>
struct Converter<cv::Size2i>
{
    static cv::Size2i doit(Json::Value const& json)
    {
        if (2 != json.size())
            throw_exception<Json::ConvertationError>("Expect list of 2 elements, but ", json.size(), " presented");

        cv::Size2i sz = {
            Converter<int>::doit(json[0]),
            Converter<int>::doit(json[1])
        };

        return sz;
    }
};

template <>
struct Converter<CameraIntrinsics>
{
    static CameraIntrinsics doit(Json::Value const& json)
    {
        CameraIntrinsics intr = {};
        json_get(json, "K", intr.K);
        json_get(json, "distortion", intr.distortion);
        json_get(json, "resolution", intr.resolution);
        return intr;
    }
};
