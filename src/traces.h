#pragma once

#include <sstream>
#include <ostream>
#include "common.h"
#include "jsonxx.h"


struct log_properties_t
{
    bool enable_errs;
    bool enable_warns;
    bool enable_info;
    bool enable_dbgs;

    log_properties_t() : 
        enable_errs(true),
        enable_warns(true),
        enable_info(false),
        enable_dbgs(false)
    {
    }
};

log_properties_t* get_log_properties();
void traces_init(jsonxx::Object const& jsoncfg);

template <typename T>
void format_value(std::ostream& s, T value)
{
    s << value;
}

template <int i, int n, typename ... T>
struct format_tuple
{
    static void doit(std::ostream& s, std::tuple<T...> const& value)
    {
        int const i_next = i < n ? i + 1 : i;

        if (i == 0)
            s << "(" << std::get<i>(value);
        else if (i == n - 1)
            s << ", " << std::get<i>(value) << ")";
        else
            s << ", " << std::get<i>(value);

        format_tuple<i_next, n, T...>::doit(s, value);
    }
};

template <int n, typename ... T>
struct format_tuple<n,n,T...> {
    static void doit(std::ostream& s, std::tuple<T...> const& value) {}
};

template <typename ... T>
void format_value(std::ostream& s, std::tuple<T...> const& value)
{
    format_tuple<0, sizeof...(T), T...>::doit(s, value);
}

inline void __print(std::ostream& s)
{
}

template <class A1, class ... Atail> 
inline void __print(std::ostream& s, A1 a, Atail ... tail)
{
    format_value(s, a);
    __print(s, tail...);
}

inline std::string format_time(int64_t usec)
{
    int64_t ms = usec / 1000;
    char buf[64];
    sprintf(buf, "%d.%03ds", int(ms / 1000), int(ms % 1000));
    return std::string(buf);
}

template <class ... Args>
inline void info_msg(Args ... args)
{
    log_properties_t* p = get_log_properties();

    if (p->enable_info)
    {
        std::stringstream ss;
        __print(ss, "[info] at ", format_time(get_time_usec()), ": ", args..., "\n");
        fprintf(stdout, "%s", ss.str().c_str());
    }
}

template <class ... Args>
inline void warn_msg(Args ... args)
{
    log_properties_t* p = get_log_properties();

    if (p->enable_warns)
    {
        std::stringstream ss;
        __print(ss, "[warning] at ", format_time(get_time_usec()), ": ", args..., "\n");
        fprintf(stdout, "%s", ss.str().c_str());
    }
}

template <class ... Args>
inline void err_msg(Args ... args)
{
    log_properties_t* p = get_log_properties();

    if (p->enable_errs)
    {
        std::stringstream ss;
        __print(ss, "[error] at ", format_time(get_time_usec()), ": ", args..., "\n");
        fprintf(stdout, "%s", ss.str().c_str());
    }
}

template <class ... Args>
inline void dbg_msg(Args ... args)
{
    log_properties_t* p = get_log_properties();

    if (p->enable_dbgs)
    {
        std::stringstream ss;
        __print(ss, "[debug] at ", format_time(get_time_usec()), ": ", args..., "\n");
        fprintf(stdout, "%s", ss.str().c_str());
    }
}

template <class ... Args>
inline void print_msg(Args ... args)
{
    std::stringstream ss;
    __print(ss, args...);
    fprintf(stdout, "%s", ss.str().c_str());
}
