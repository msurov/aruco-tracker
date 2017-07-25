#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <list>
#include <mutex>
#include <condition_variable>

class ImageSaver
{
private:
    struct image_data
    {
        cv::Mat     img;
        std::string path;
        std::string type;
        int         downscale;
    };

    std::list<image_data>   m_frames;

    std::mutex              m_tasks_mutex;
    std::thread             m_call_thread;
    bool                    m_stop;
    std::condition_variable m_signal;

    void save_frame(image_data& f);
    void loop();

public:
    ImageSaver();
    ~ImageSaver();

    void push(
        std::string const& path,
        cv::Mat const& img,
        std::string const& type = "auto",    // bgr | gray | bggr | raw
        int downscale = 1
    );
};
