#pragma once

#include <fstream>
#include <stdexcept>
#include <json/json.h>
#include <misc/traces.h>
#include <misc/throws.h>


template <typename T> struct JsonType { 
    using type = T;
    static const Json::ValueType id = Json::ValueType::nullValue;
    static std::string name() { return std::string("undefined"); };
    static T convert(Json::Value const&);
};
template <> struct JsonType<double> {
    using type = double;
    static const Json::ValueType id = Json::ValueType::realValue; 
    static std::string name() { return std::string("double"); }
    static double convert(Json::Value const& v) { return v.asDouble(); }
};
template <> struct JsonType<float> {
    using type = float;
    static const Json::ValueType id = Json::ValueType::realValue; 
    static std::string name() { return std::string("double"); }
    static float convert(Json::Value const& v) { return float(v.asDouble()); }
};
template <> struct JsonType<int> { 
    using type = int;
    static const Json::ValueType id = Json::ValueType::intValue;
    static std::string name() { return std::string("int"); };
    static int convert(Json::Value const& v) { return v.asInt(); };
};
template <> struct JsonType<int64_t> {
    using type = Json::Int64;
    static const Json::ValueType id = Json::ValueType::intValue;
    static std::string name() { return std::string("int64"); };
    static int64_t convert(Json::Value const& v) { return v.asInt64(); };
};
template <> struct JsonType<uint> { 
    using type = Json::UInt; 
    static const Json::ValueType id = Json::ValueType::uintValue; 
    static std::string name() { return std::string("uint"); };
    static uint convert(Json::Value const& v) { return v.asUInt(); };
};
template <> struct JsonType<uint64_t> { 
    using type = Json::UInt64;
    static const Json::ValueType id = Json::ValueType::uintValue; 
    static std::string name() { return std::string("uint64"); };
    static uint64_t convert(Json::Value const& v) { return v.asUInt64(); };
};
template <> struct JsonType<std::string> { 
    using type = std::string;
    static const Json::ValueType id = Json::ValueType::stringValue; 
    static std::string name() { return std::string("string"); };
    static std::string convert(Json::Value const& v) { return v.asString(); };
};
template <> struct JsonType<bool> {
    using type = bool;
    static const Json::ValueType id = Json::ValueType::booleanValue; 
    static std::string name() { return std::string("bool"); };
    static bool convert(Json::Value const& v) { return v.asBool(); };
};
template <> struct JsonType<Json::Value> { 
    using type = Json::Value;
    static const Json::ValueType id = Json::ValueType::objectValue; 
    static std::string name() { return std::string("object"); };
    static Json::Value convert(Json::Value const& v) { return v; };
};

template <typename T>
inline T json_get(Json::Value const& cfg, std::string const& name, T const& default_value)
{
    if (!cfg.isMember(name))
        return default_value;

    auto const& value = cfg[name];
    if (!value.isConvertibleTo(JsonType<T>::id))
        throw_runtime_error("json entry ", name, " is not convertible to ", JsonType<T>::name());

    return JsonType<T>::convert(value);
}

template <typename T=Json::Value>
inline T json_get(Json::Value const& v, std::string const& name)
{
    if (!v.isMember(name))
        throw_runtime_error("json ", v.toStyledString(), " does not contain entry ", name);

    auto const& value = v[name];
    if (!value.isConvertibleTo(JsonType<T>::id))
        throw_runtime_error("json entry ", name, " is not convertible to ", JsonType<T>::name());

    return JsonType<T>::convert(value);
}

template <typename T>
inline bool json_has(Json::Value const& v, std::string const& name)
{
    if (!v.isMember(name))
        return false;

    auto const& value = v[name];
    if (!value.isConvertibleTo(JsonType<T>::id))
        return false;

    return true;
}

template <>
inline Json::Value json_get<Json::Value>(Json::Value const& v, std::string const& name)
{
    if (!v.isMember(name))
        throw_runtime_error("json ", v.toStyledString(), " does not contain entry ", name);

    return v[name];
}

template <typename T>
inline T json_get(Json::Value const& v, std::string const& name, T const& vmin, T const& vmax)
{
    T const& ans = json_get<T>(v, name);
    if (!in_range(ans, vmin, vmax))
        throw_runtime_error("json entry ", name, " is out of range  ", vmin, "...", vmax);
    return ans;
}

template <typename T=Json::Value>
inline std::vector<T> json_get_vec(Json::Value const& v, std::string const& name)
{
    if (!v.isMember(name))
        throw_runtime_error("json ", v.toStyledString(), " does not contain entry ", name);

    Json::Value json_arr = v[name];
    const int n = json_arr.size();
    std::vector<T> vec(n);

    for (int i = 0; i < n; ++ i)
        vec[i] = JsonType<T>::convert(json_arr[i]);

    return vec;
}

template <typename T, std::size_t N>
inline std::array<T, N> json_get_arr(Json::Value const& v, std::string const& name)
{
    if (!v.isMember(name))
        throw_runtime_error("json ", v.toStyledString(), " does not contain entry ", name);

    Json::Value json_arr = v[name];
    const int n = json_arr.size();
    if (n != N)
        throw_runtime_error("json array ", json_arr.toStyledString(), " is of different length, expected ", N);

    std::array<T, N> arr;
    for (int i = 0; i < n; ++ i)
        arr[i] = JsonType<T>::convert(json_arr[i]);

    return arr;
}

template <typename T>
inline void json_append(Json::Value& entry, std::string const& name, T const& value)
{
    typename JsonType<T>::type _value(value);
    entry[name] = _value;
}

inline void json_append(Json::Value& entry, std::string const& name, char const* value)
{
    entry[name] = value;
}

template <typename Arr>
inline void json_append_arr(Json::Value& entry, std::string const& name, Arr const& vec)
{
    Json::Value json_vec;
    const int n = vec.size();

    for (int i = 0; i < n; ++ i)
    {
        typename JsonType<typename Arr::value_type>::type elem(vec[i]);
        json_vec[i] = elem;
    }

    entry[name] = json_vec;
}

inline std::string json_pack(Json::Value const& json)
{
    return json.toStyledString();
}

inline Json::Value json_unpack(std::string const& s)
{
    std::istringstream ss(s);
    Json::Value json;
    ss >> json;
    return json;
}

inline Json::Value json_load(std::string const& filepath)
{
    Json::Value json;
    std::ifstream file(filepath);
    file >> json;
    return json;
}
