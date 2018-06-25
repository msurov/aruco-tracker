#pragma once

#include <array>
#include <vector>


template <typename... T>
inline constexpr auto make_array(T&&... values) ->
        std::array<
            typename std::decay<typename std::common_type<T...>::type>::type,
            sizeof...(T)
        >
{
    return std::array<
        typename std::decay<typename std::common_type<T...>::type>::type,
        sizeof...(T)
    >{{ std::forward<T>(values)... }};
}

template<class T> struct identity { using type = T; };
template<class D, class... Ts>
struct ret : identity<D> {};
template<class... Ts>
struct ret<void, Ts...> : std::common_type<Ts...> {};
template<class D, class... Ts>
using ret_t = typename ret<D, Ts...>::type;

template<class D = void, class... Ts>
std::vector<ret_t<D, Ts...>> make_vector(Ts&&... args) {
    std::vector<ret_t<D, Ts...>>  ret;
    ret.reserve(sizeof...(args));
    using expander = int[];
    (void) expander{ (ret.emplace_back(std::forward<Ts>(args)), 0)..., 0 };
    return ret;
}
