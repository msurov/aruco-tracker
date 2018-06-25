#pragma once

#include <string>
#include <fstream>
#include "throws.h"


inline std::string read_all(std::string const& file)
{
    std::ifstream f(file);
    if (!f.good())
        throw_runtime_error("can't open ", file);

    f.seekg(0, f.end);
    int length = f.tellg();
    f.seekg(0, f.beg);

    if (length < 0)
        throw_runtime_error("failed requiring file size");

    std::string data;
    data.resize(length);
    f.read(&data[0], data.size());
    if (!f.good())
        throw_runtime_error("failed to read file ", file);

    return data;
}

inline void write_file(std::string const& filepath, std::string const& data)
{
    std::ofstream f(filepath);
    f.write(data.data(), data.size());
    f.flush();
}
