#pragma once

#include <cppmisc/json.h>
#include "cvmath.h"
#include "camera_parameters.h"
#include "quat.h"
#include "transforms.h"


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

        return {
            Converter<int>::doit(json[0]),
            Converter<int>::doit(json[1])
        };
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

template <typename T>
struct Converter<QuatT<T>>
{
    static QuatT<T> doit(Json::Value const& json)
    {
        QuatT<T> q;

        if (json.isArray() && json.size() == 4)
        {
            q.w() = Converter<T>::doit(json[0]);
            q.x() = Converter<T>::doit(json[1]);
            q.y() = Converter<T>::doit(json[2]);
            q.z() = Converter<T>::doit(json[3]);
        }
        else if (json_has(json, "w"))
        {
            json_get(json, "w", q.w());
            json_get(json, "x", q.x());
            json_get(json, "y", q.y());
            json_get(json, "z", q.z());
        }
        else
        {
            throw_runtime_error("can't interpret as json entry as quaternion");
        }

        return q;
    }
};

template <>
struct Converter<Pose>
{
    static Pose doit(Json::Value const& json)
    {
        Pose pose;
        json_get(json, "position", pose.p);

        const auto json_orientation = json_get(json, "orientation");
        if (json_orientation.size() == 3)
        {
            pose.r = Converter<cv::Vec3d>::doit(json_orientation);
        }
        else if (json_orientation.size() == 4)
        {
            const auto q = Converter<Quat>::doit(json_orientation);
            pose.r = rodrigues(q);
        }
        return pose;
    }
};
