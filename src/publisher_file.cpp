#include <memory>
#include "publisher.h"
#include <cppmisc/throws.h>
#include <cppmisc/traces.h>
#include <cppmisc/json.h>
#include <iostream>
#include <fstream>


class FilePublisher : public Publisher
{
private:
    std::ostream* _out;
    std::fstream _file;
    std::string _buf;

public:
    FilePublisher(std::string const& filepath);
    ~FilePublisher();
    bool start() override;
    void term() override;
    bool publish(int64_t t, std::vector<Marker> const& markers) override;
    static PublisherPtr create(Json::Value const& config);
};

FilePublisher::FilePublisher(std::string const& filepath)
{
    if (filepath == "stdout")
    {
        _out = &std::cout;
    }
    else
    {
        _file.open(filepath, std::ios_base::out);
        _out = &_file;
    }

    if (!_out || _out->bad())
        throw_runtime_error("can't open file ", filepath, " for writing");
    
    _buf.reserve(1024);
}

FilePublisher::~FilePublisher()
{
    term();
}

bool FilePublisher::start()
{
    return true;
}

void FilePublisher::term()
{
    if (_file.is_open())
        _file.close();
    _out = nullptr;
}

bool FilePublisher::publish(int64_t ts, std::vector<Marker> const& markers)
{
    const int nelems = markers.size();
    char const fmt[] = "id=%d; p=[%f,%f,%f]; r=[%f,%f,%f];\n";
    int bufsz = nelems * (sizeof(fmt) + 16 * 7) + 1;
    _buf.resize(bufsz);
    int used = 0;
    char* ptr = &_buf[0];

    for (auto const& marker : markers)
    {
        auto const& p = marker.world_marker_pose.p;
        auto const& r = marker.world_marker_pose.r;
        int n = snprintf(ptr, bufsz - used, fmt, marker.id, p(0), p(1), p(2), r(0), r(1), r(2));
        if (n <= 0)
            return false;
        ptr += n;
        used += n;
    }

    _out->write(&_buf[0], used);
    if (_out->bad())
        return false;
    return true;
}

PublisherPtr create_publisher_file(Json::Value const& cfg)
{
    std::string file = json_get<std::string>(cfg, "file");
    auto ptr = std::make_shared<FilePublisher>(file);
    return std::static_pointer_cast<Publisher>(ptr);
}
