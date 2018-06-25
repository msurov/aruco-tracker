#include <stdexcept>
#include <unistd.h>
#include <cstdio>
#include "traces.h"
#include <syslog.h>
#include <fcntl.h>
#include <iostream>


static const std::string __color_red = "\x1b[31m";
static const std::string __color_green = "\x1b[32m";
static const std::string __color_yellow = "\x1b[33m";
static const std::string __color_blue = "\x1b[34m";
static const std::string __color_magenta = "\x1b[35m";
static const std::string __color_cyan = "\x1b[36m";
static const std::string __color_gray = "\x1b[90m";
static const std::string __reset_color = "\x1b[0m";

#if defined(GIT_VERSION)
    static const std::string __app_version = GIT_VERSION;
#else
    static const std::string __app_version = "undef";    
#endif

// TODO: configurable cout/syslog

Traces::Traces() :
    _errs(true),
    _warns(true),
    _info(true),
    _dbgs(true),
    info_tag(__color_cyan + "[info]" + __reset_color),
    debug_tag(__color_gray + "[debug]" + __reset_color),
    error_tag(__color_red + "[error]" + __reset_color),
    warning_tag(__color_yellow + "[warn]" + __reset_color)
    // info_tag("[info]"),
    // debug_tag("[debug]"),
    // error_tag("[error]"),
    // warning_tag("[warn]")
{
    // openlog("robot_driver", LOG_NDELAY | LOG_PID, LOG_USER);
    // _file = fopen("/tmp/tkrd.log", "w");
    _file = stdout;
    std::string s = "robot driver " + __app_version + " launched\n";
    fwrite(s.data(), 1, s.size(), _file);
}

Traces::~Traces()
{
    // closelog();
    std::string s = "log done\n";
    fwrite(s.data(), 1, s.size(), _file);
    // fflush(_file);
    // fclose(_file);
    _file = nullptr;
}

Traces* Traces::instance()
{
    static Traces traces;
    return &traces;
}

void Traces::enable(bool warnings, bool errors, bool info, bool dbgs)
{
    _warns = warnings;
    _errs = errors;
    _info = info;
    _dbgs = dbgs;
}

void Traces::write_stdout(std::string const& s)
{
    int ans = fwrite(s.data(), 1, s.size(), _file);
    if (ans != (int)s.size())
    {
        std::cout << "failed to write to file: " << s << std::endl;
        return;
    }
    // syslog(LOG_INFO, "%s", s.c_str());
}

void Traces::write_stderr(std::string const& s)
{
    int ans = fwrite(s.data(), 1, s.size(), _file);
    if (ans != (int)s.size())
    {
        std::cout << "failed to write to file: " << s << std::endl;
        return;
    }
    // syslog(LOG_ERR, "%s", s.c_str());
    std::cerr << s << std::endl;
}

void Traces::write_stddbg(std::string const& s)
{
    int ans = fwrite(s.data(), 1, s.size(), _file);
    if (ans != (int)s.size())
    {
        std::cout << "failed to write to file: " << s << std::endl;
        return;
    }
    // syslog(LOG_DEBUG, "%s", s.c_str());
}
