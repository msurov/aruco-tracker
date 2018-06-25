#pragma once

#include <thread>
#include <functional>
#include <stdint.h>
#include "time.h"


inline bool set_thread_rt_priotiy(pthread_t thread, int priotity)
{
    if (thread == pthread_t(-1))
        thread = pthread_self();

    int policy = SCHED_RR; // SCHED_FIFO;
    sched_param param;
    param.sched_priority = priotity;
    return pthread_setschedparam(thread, policy, &param) == 0;
}

class LoopRate
{
private:
    int64_t _t_prev;
    int64_t _interval;

public:
    LoopRate(int64_t interval_usec) : _t_prev(epoch_usec()), _interval(interval_usec) {}
    ~LoopRate() {}

    inline void wait()
    {
        int64_t t = epoch_usec();

        while (t < _t_prev + _interval)
        {
            sleep_usec(_t_prev + _interval - t);
            t = epoch_usec();
        }

        _t_prev = t;
    }
};

class PeriodicCall;
typedef std::shared_ptr<PeriodicCall> PeriodicCallPtr;

class PeriodicCall
{
public:
    typedef std::function<void(void)> callback;

private:
    callback _cb;
    int64_t _interval;
    std::thread _thread;
    bool _stop;

    void handler()
    {
        LoopRate rate(_interval);

        while (!_stop)
        {
            _cb();
            rate.wait();
        }
    }

public:
    PeriodicCall() = delete;
    PeriodicCall(PeriodicCall&&) = delete;
    PeriodicCall(PeriodicCall const&) = delete;

    PeriodicCall(callback cb, int64_t interval_usec) : 
        _cb(std::move(cb)), 
        _interval(interval_usec),
        _stop(true)
    {
        start();
    }

    ~PeriodicCall()
    {
        stop();
    }

    inline void start()
    {
        if (!_stop)
            return;

        _stop = false;
        _thread = std::thread(&PeriodicCall::handler, this);
    }

    inline void stop()
    {
        _stop = true;
        if (_thread.joinable())
            _thread.join();
    }

    inline std::thread::native_handle_type native_handle()
    {
        return _thread.native_handle();
    }
};
