#include <opencv2/imgcodecs.hpp>
#include <cppmisc/task_queue.h>
#include <cppmisc/files.h>
#include <cppmisc/misc.h>
#include <cppmisc/traces.h>
#include "imgdump.h"


void imsave(std::string const& path, cv::Mat const& im)
{
    auto ext = getext(path);
    if (one_of(ext, {".bmp", ".png", ".jpg", ".jpeg"}))
    {
        bool ok = cv::imwrite(path, im);
        if (!ok)
            err_msg("failed to save image ", path);
    }
    else if (ext == ".raw")
    {
        const int Ny = im.rows;
        const int Nx = im.cols;
        FILE* f = fopen(path.c_str(), "w");

        {
            std::stringstream ss;
            ss << "@raw, nx=" << Nx << ", ny=" << Ny;
            std::string s = ss.str();
            int n = s.size();
            fwrite(s.c_str(), 1, n + 1, f);
        }

        for (int y = 0; y < Ny; ++ y)
        {
            uchar const* p = im.ptr<uchar>(y);
            fwrite(p, 1, Nx, f);
        }
    }
    else
    {
        err_msg("failed to save image ", path, ", unknown file type");
    }

    dbg_msg("saved ", path);
}

class ImgDump
{
private:
    TaskQueue _tasks;
    std::string _path;

public:
    ImgDump(Json::Value const& cfg)
    {
        _path = json_get<std::string>(cfg, "directory");
        if (!dir_exists(_path))
            throw_runtime_error("the directory ", _path, " doesn't exist");
    }

    void push(char const* name, cv::Mat const& im)
    {
        cv::Mat imcopy = im.clone();
        std::string impath = format(_path, "/", name);
        auto f = [impath,imcopy]() { imsave(impath, imcopy); };
        _tasks.emplace(f);
    }
};


std::unique_ptr<ImgDump> __imgdump;

void imgdump::init(Json::Value const& cfg)
{
    __imgdump.reset(new ImgDump(cfg));
}

void imgdump::push(char const* name, cv::Mat const& im)
{
    if (!__imgdump)
        return;
    __imgdump->push(name, im);
}

void imgdump::push(std::string const& name, cv::Mat const& im)
{
    return imgdump::push(name.c_str(), im);
}

bool imgdump::enabled()
{
    if (__imgdump)
        return true;
    return false;
}
