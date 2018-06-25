#pragma once

#include <stdexcept>
// #include <stdnoreturn.h> gcc bug!
#include "formatting.h"

#define noreturn_attr __attribute__ ((__noreturn__))


template <class Exception, class ... Args>
noreturn_attr inline void throw_exception(Args ... args)
{
    auto const& s = format_args(args...);
    throw Exception(s);
}

template <class ... Args>
noreturn_attr inline void throw_runtime_error(Args ... args)
{
    throw_exception<std::runtime_error>(args...);
}

template <class ... Args>
noreturn_attr inline void throw_invalid_arg(Args ... args)
{
    throw_exception<std::invalid_argument>(args...);
}
