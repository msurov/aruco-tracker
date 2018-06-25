#pragma once
#include <opencv2/opencv.hpp>


template <int ... S>
struct seq {};

template <int N, int ... S>
struct gen : gen<N - 1, N, S...>
{
};

template <int ... S>
struct gen<0, S...>
{
    typedef seq<S...> type;
};

template <typename F, int ... S, typename ... T>
inline void __call_func(F const& f, seq<S...>, std::tuple<T...> t)
{
    f(std::get<S - 1>(t)...);
}

template <typename F, typename ... T>
inline void call_func(F const& func, std::tuple<T...> args)
{
    typename gen<sizeof...(T)>::type s;
    __call_func(func, s, args);
}

template <class F, typename ... Args>
class LoopBody : public cv::ParallelLoopBody
{
private:
    F m_f;
    std::tuple<Args...> m_args;

public:

    LoopBody(LoopBody const&) = delete;

    LoopBody(F& f, int nrows, Args... args) :
        m_f(f),
        m_args(std::make_tuple(args...))
    {
        int ntasks = nrows / 16;
    }

    virtual ~LoopBody()
    {
    }

    virtual void operator() (cv::Range const& range) const
    {
        call_func(m_f, std::tuple_cat(std::make_tuple(range), m_args));
    }
};

template <typename F, typename ... Args>
inline void run_parallel(F f, int nrows, Args... args)
{
    LoopBody<F, Args...> loop(f, nrows, args...);
    parallel_for_(cv::Range(0, nrows), loop, 16.0);
}
