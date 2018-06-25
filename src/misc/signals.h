#pragma once

#include <functional>

class SysSignals
{
public:
    typedef std::function<void(void)> sighandler_t;

private:
    sighandler_t _sigint_handler;
    sighandler_t _sigterm_handler;
    sighandler_t _sighup_handler;

    static void handle(int sig);

public:
    SysSignals(SysSignals&&) = delete;
    SysSignals(SysSignals const&) = delete;
    SysSignals();
    ~SysSignals();

    void set_sigint_handler(sighandler_t handler);
    void set_sigterm_handler(sighandler_t handler);
    void set_sighup_handler(sighandler_t handler);
    static SysSignals* instance();
};
