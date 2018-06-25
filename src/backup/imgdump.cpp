#include "image_saver.h"
#include "imgdump.h"


class ImgDump
{
private:
    ImageSaver saver;
    std::string path;
    bool active;

    ImgDump() : path(), active(false) {}

public:
    ~ImgDump() {}

    static ImgDump& instance()
    {
        static ImgDump inst;
        return inst;
    }

    inline void init(jsonxx::Object const& cfg)
    {
        auto logger = json_get<jsonxx::Object>(cfg, "imgdump");
        if (logger.has<jsonxx::String>("img_dump_dir"))
        {
            path = json_get<jsonxx::String>(logger, "img_dump_dir");
            active = true;
        }
        else
        {
            active = false;
        }
    }

    inline void doit(std::string const& name, cv::Mat const& img, std::string const& type)
    {
        if (!active)
            return;

        saver.push(path + "/" + name, img, type);
    }
};

void imgdump_init(jsonxx::Object const& cfg)
{
    ImgDump::instance().init(cfg);
}

void imgdump(std::string const& name, cv::Mat const& img, std::string const& type)
{
    ImgDump::instance().doit(name, img, type);
}
