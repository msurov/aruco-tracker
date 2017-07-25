#include <opencv2/opencv.hpp>
#include <assert.h>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <initializer_list>
#include "common.h"
#include "traces.h"
#include "image_saver.h"


using namespace std;
using namespace cv;


template <typename T>
inline bool one_of(string const& src, initializer_list<T> const& list)
{
    for (auto const& e: list)
    {
        if (compare_case_insensitive(src, e) == 0)
            return true;
    }

    return false;
}


void ImageSaver::save_frame(image_data& f)
{
    if (f.type == "bgr" || f.type == "gray")
    {
        imwrite(f.path, f.img);
    }
    else if (f.type == "bggr")
    {
        string ext = getext(f.path);

        if (one_of(ext, {".bmp", ".png", ".jpg"}))
        {
            Mat img;
            // demosaicing(f.img, img, cv::COLOR_BayerRG2BGR);
            cvtColor(f.img, img, cv::COLOR_BayerRG2BGR);
            imwrite(f.path, img);
        }
        else if (one_of(ext, {".bin", ".raw"}))
        {
            std::ofstream file(f.path.c_str(), std::ios_base::out | std::ios_base::binary);
            auto line_size = f.img.elemSize() * f.img.cols;

            for (int y = 0; y < f.img.rows; ++y)
            {
                uchar const* line = f.img.ptr<uchar>(y);
                file.write((char const*)line, line_size);
            }

            file.close();
        }
    }
    else if (f.type == "raw")
    {
        std::ofstream file(f.path.c_str(), std::ios_base::out | std::ios_base::binary);
        auto line_size = f.img.elemSize() * f.img.cols;

        for (int y = 0; y < f.img.rows; ++y)
        {
            uchar const* line = f.img.ptr<uchar>(y);
            file.write((char const*)line, line_size);
        }

        file.close();
    }
    else
    {
        err_msg("ImageSaver: unknown image type");
    }
}

void ImageSaver::loop()
{
    while (true)
    {
        image_data f;

        {
            unique_lock<mutex> lock(m_tasks_mutex);

            while (m_frames.empty() && !m_stop)
                m_signal.wait(lock);

            if (m_frames.empty() && m_stop)
                break;

            f = m_frames.front();
            m_frames.pop_front();
        }

        save_frame(f);
    }
}


ImageSaver::ImageSaver()
{
    m_stop = false;
    m_call_thread = thread(&ImageSaver::loop, this);
}

ImageSaver::~ImageSaver()
{
    {
        unique_lock<mutex> lock(m_tasks_mutex);
        m_stop = true;
    }

    m_signal.notify_all();
    m_call_thread.join();
}

void ImageSaver::push(
    string const& path,
    Mat const& img,
    string const& type,
    int downscale
    )
{
    unique_lock<mutex> lock(m_tasks_mutex);
    string _type(type);

    if (type == "auto")
    {
        if (one_of(getext(path), {".bin", ".raw"}))
            _type = "raw";
        else if (img.channels() == 1)
            _type = "gray";
        else if (img.channels() == 3)
            _type = "bgr";
        else
            throw invalid_argument("can't recognize image format");
    }

    Mat img2;

    if (downscale == 1)
        img.copyTo(img2);
    else
        resize(img, img2, Size(), 1.0 / downscale, 1.0 / downscale, INTER_NEAREST);
        // resize(img, img2, Size(), 1.0 / downscale, 1.0 / downscale, INTER_CUBIC);

    image_data f = { img2, path, _type, downscale };
    m_frames.push_back(f);
    m_signal.notify_all();
}
