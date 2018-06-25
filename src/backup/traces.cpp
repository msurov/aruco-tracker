#include <stdexcept>
#include <cstdio>
#include "traces.h"


log_properties_t* get_log_properties()
{
    static log_properties_t instance;
    return &instance;
}

void traces_init(jsonxx::Object const& jsoncfg)
{
    if (!jsoncfg.has<jsonxx::Object>("logger"))
        throw std::runtime_error("config doesn't have 'logger' entry");

    auto const& json_logger = jsoncfg.get<jsonxx::Object>("logger");
    log_properties_t* p = get_log_properties();

    if (json_logger.has<jsonxx::Array>("enable_print"))
    {
        auto const enable_prints = json_logger.get<jsonxx::Array>("enable_print");

        bool enable_info = false;
        bool enable_errs = false;
        bool enable_warns = false;
        bool enable_dbgs = false;

        for (int i = 0; i < (int)enable_prints.size(); ++i)
        {
            std::string s = enable_prints.get<jsonxx::String>(i, "incorrect");

            if (s == "all")
                enable_info = enable_errs = enable_dbgs = enable_warns = true;
            else if (s == "warnings")
                enable_warns = true;
            else if (s == "errors")
                enable_errs = true;
            else if (s == "infos")
                enable_info = true;
            else if (s == "debugs")
                enable_dbgs = true;
            else
                throw std::runtime_error("config incorrect value of variable 'debug.print'");
        }

        p->enable_info = enable_info;
        p->enable_errs = enable_errs;
        p->enable_warns = enable_warns;
        p->enable_dbgs = enable_dbgs;
    }
}
