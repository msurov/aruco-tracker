#pragma once
#include <vector>
#include <string>
#include <tuple>
#include <map>
#include <stdexcept>
#include <iomanip>
#include <memory>
#include <sstream>
#include <initializer_list>


struct make_arg
{
    std::vector<std::string> keys;
    std::string name;
    std::string help;
    bool required;
    std::string defval;

    make_arg(
        std::string const& key, 
        std::string const& name, 
        std::string const& help = "", 
        std::string const& default_value = "", 
        bool required = false
    ) :
        keys({ key }), name(name), help(help), required(required), defval(default_value)
    {
    }

    make_arg(
        std::initializer_list<char const*> const& keys, 
        std::string const& name, 
        std::string const& help = "",
        std::string const& default_value = "",
        bool required = false
    ) :
        name(name), help(help), required(required), defval(default_value)
    {
        this->keys.reserve(keys.size());
        for (auto s : keys)
            this->keys.push_back(s);
    }
};

std::tuple<std::string, std::string> split(std::string const& s, std::string const& symbols)
{
    auto n = s.find_first_of(symbols);
    if (n == std::string::npos)
        return std::make_tuple(s, "");
    return std::make_tuple(s.substr(0, n), s.substr(n + 1));
}

std::tuple<std::string, std::string> split_name(std::string const& path)
{
    auto n = path.find_last_of("\\/");
    if (n == path.npos)
        return std::make_tuple("", path);
    return std::make_tuple(path.substr(0, n + 1), path.substr(n + 1));
}

struct make_arg_list
{
private:
    std::string program_name;
    std::vector<std::shared_ptr<make_arg>> arg_list;
    std::map<std::string, std::shared_ptr<make_arg>> args_map;

    void add_arg(make_arg const& arg)
    {
        arg_list.push_back(std::make_shared<make_arg>(arg));

        for (auto const& key : arg_list.back()->keys)
        {
            if (args_map.find(key) != args_map.end())
                throw std::invalid_argument("the multiple definition of the key " + key);

            args_map[key] = arg_list.back();
        }
    }

    inline void init(std::initializer_list<make_arg> const& args)
    {
        for (auto const& arg : args)
            add_arg(arg);
    }

public:
    make_arg_list()
    {
        program_name = "program";
    }

    /*
     * typical usage:
     *
     *   make_arg_list args({
     *       {string key, string name, string help_description, string default_value, bool required},
     *       {...}
     *   });
     *   map = args.parse(argc, argv);
     */
    make_arg_list(std::initializer_list<make_arg> const& args)
    {
        program_name = "program";
        init(args);
    }

    std::string help_message() const
    {
        std::stringstream ss;

        ss << "usage: " << std::endl;
        ss << "    " << program_name << " ";
        for (size_t i = 0; i < arg_list.size(); ++i)
        {
            if (arg_list[i]->required)
                ss << arg_list[i]->keys[0] << "=" << arg_list[i]->name << " ";
            else
                ss << "[" << arg_list[i]->keys[0] << "] ";
        }
        ss << std::endl << std::endl;

        ss << "arguments:" << std::endl;
        std::vector<std::string> keys_list;
        std::vector<std::string> help_list;

        keys_list.reserve(args_map.size());
        help_list.reserve(args_map.size());

        for (auto const& e : arg_list)
        {
            std::string keys;
            for (size_t i = 0; i < e->keys.size(); ++i)
            {
                if (i == e->keys.size() - 1)
                    keys += e->keys[i];
                else
                    keys += e->keys[i] + ", ";
            }

            keys_list.push_back(keys);
            help_list.push_back(e->help);
        }

        std::string::size_type max_len = 0;
        for (auto const& s : keys_list)
            max_len = std::max(s.size(), max_len);

        for (size_t i = 0; i < keys_list.size(); ++i)
            ss << "    " << std::left << std::setw(max_len + 2) << keys_list[i] << help_list[i] << std::endl;

        return ss.str();
    }

    std::map<std::string, std::string> parse(int argc, char* argv[])
    {
        std::map<std::string, std::string> result;

        if (argc == 0)
            return result;

        std::tie(std::ignore, program_name) = split_name(argv[0]);
        int default_idx = 0;

        for (auto const& e : args_map)
        {
            if (e.second->defval != "")
                result.emplace(e.second->name, e.second->defval);
        }

        for (int i = 1; i < argc; ++i)
        {
            char const* arg = argv[i];
            auto ans = split(arg, "=");

            if (std::get<1>(ans) == "")
            {
                std::string key = std::get<0>(ans);

                if (args_map.find(key) == args_map.end())
                {
                    // format: value1 value2 ...
                    std::string name = "arg-" + std::to_string(default_idx);
                    ++ default_idx;
                    result[name] = std::get<0>(ans);
                }
                else
                {
                    // format: key1 value1 key2 value2 ...
                    if (i == argc - 1)
                        throw std::invalid_argument("argument " + key + " is undefined");

                    ++i;
                    std::string name = args_map.at(key)->name;
                    result[name] = argv[i];
                }
            }
            // format: key1=value1 key2=value2 ...
            else
            {
                std::string key = std::get<0>(ans);
                if (args_map.find(key) == args_map.end())
                    throw std::invalid_argument("don't know the flag: " + std::get<0>(ans));

                std::string name = args_map.at(key)->name;
                result[name] = std::get<1>(ans);
            }
        }

        for (auto const& e : args_map)
        {
            if (e.second->required)
            {
                if (result.find(e.second->name) == result.end())
                    throw std::invalid_argument("property " + e.second->name + " is not specified");
            }
        }

        return result;
    }
};
