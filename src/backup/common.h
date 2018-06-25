#pragma once

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <memory>
#include "jsonxx.h"

#ifdef _MSC_VER
    #include <Windows.h>
    #include <stdint.h>

#else // gcc
    #include <sys/time.h>
    #include <unistd.h>

#endif 

#define throw_if(cond, excpt) if ((cond)) { throw (excpt); }

/*
 * arithmetric
 */
template <typename T>
inline T clamp(T x, T a, T b)
{
    return 
        x < a ? a : 
        x > b ? b : x;
}

template <typename T>
inline T in_diap(T x, T a, T b)
{
    return x > a && x < b;
}

template <typename T>
inline T square(T x)
{
    return x * x;
}

static const double _PI_4 = std::atan(1.0f);
static const double _PI_2 = 2 * _PI_4;
static const double _PI = 4 * _PI_4;


template <typename Arr, typename Fun>
inline int argmin(Arr const& arr, Fun const& fun)
{
    int minidx = 0;
    auto minval = fun(arr[0]);

    for (int i = 1; i < arr.size(); ++i)
    {
        auto val = fun(arr[i]);
        if (val < minval)
        {
            minidx = i;
            minval = val;
        }
    }

    return minidx;
}

template <typename T>
inline T minval(std::vector<T> const& arr)
{
    int idx = argmin(arr, [](T const& elem) { return elem; });
    return arr[idx];
}

template <typename T>
inline T maxval(std::vector<T> const& arr)
{
    int idx = argmin(arr, [](T const& elem) { return -elem; });
    return arr[idx];
}


/*
 * path manipulation
 */

std::string stripstr(std::string const& s);
std::tuple<std::string, std::string> splitname(std::string const& path);
std::tuple<std::string, std::string> splitext(std::string const& path);
std::string getname(std::string const& path);
std::string getext(std::string const& path);
int compare_case_insensitive(std::string const& s1, std::string const& s2);
std::string get_digits_substr(std::string const& s);
bool endswith(std::string const& str, std::string const& ending);

/*
 * time & perf counter
 */
#ifdef _MSC_VER
    inline int64_t __epoch_usec()
    {
        LARGE_INTEGER n, f;
        QueryPerformanceCounter(&n);
        QueryPerformanceFrequency(&f);
        return n.QuadPart * 1000000i64 / f.QuadPart;
    }

#else
    inline int64_t __epoch_usec()
    {
        auto epoch = std::chrono::high_resolution_clock::now().time_since_epoch();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
        return us.count();
    }

#endif

int64_t __beginning_epoch_usec();

inline int64_t get_time_usec()
{
    return __epoch_usec() - __beginning_epoch_usec();
}

template <typename T>
inline void sleep_usec(T const& usec)
{
    auto dur = std::chrono::microseconds((int64_t)usec);
    std::this_thread::sleep_for(dur);
}

inline void sleep_until(int64_t t)
{
    auto t1 = get_time_usec();
    if (t > t1)
        sleep_usec(t - t1);
}

typedef std::function<void(void)> signal_handler_t;
typedef std::shared_ptr<signal_handler_t> ptr_signal_handler_t;

void set_sigint_handler(ptr_signal_handler_t ptr_handler);
void set_sigterm_handler(ptr_signal_handler_t ptr_handler);

bool match_mask(std::string const& filename, std::string const& mask);
std::vector<std::string> get_files(std::string const& mask);
