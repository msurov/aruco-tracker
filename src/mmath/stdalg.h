#pragma once


template <typename T>
inline T clip(T const& a, T const& amin, T const& amax)
{
    return 
        a < amin ? amin : 
        a > amax ? amax : 
        a;
}

template <typename T>
inline T sign(T const& a)
{
    return a > static_cast<T>(0) ? static_cast<T>(1) : static_cast<T>(-1);
}
