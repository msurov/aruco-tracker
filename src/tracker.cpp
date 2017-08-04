#include "traces.h"
#include "camera.h"
#include "parse_args.h"
#include "common.h"
#include "imgdump.h"
#include "aruco_detector.h"

using namespace std;
using namespace cv;


static bool brun = false;
static unique_ptr<ArucoDetector> p_detector;

static void camera_handler(uint8_t const* data, int Nx, int Ny, int64_t frame_timestamp)
{
    if (!brun)
        return;

    try
    {
        if (p_detector)
        {
            Mat im(Ny, Nx, CV_8U, (void*)data);
            map<int, polygon_t> markers;
            p_detector->find_markers(im, markers);

            // TODO
        }
    }
    catch (exception& e)
    {
        err_msg(e.what());
    }
    catch (...)
    {
        err_msg("caught unknown exception");
    }
}

void run_tracker(jsonxx::Object const& cfg)
{
    auto f = []() { 
        brun = false; 
    };
    auto pf = make_shared<signal_handler_t>(f);
    set_sigint_handler(pf);
    set_sigterm_handler(pf);

    p_detector = get_aruco_detector(cfg);

    init_camera(cfg);
    get_camera()->set_handler(camera_handler);

    brun = true;
    get_camera()->run();
    while (brun)
        sleep_usec(1e+6);
    get_camera()->stop();
}

int main(int argc, char* argv[])
{
    make_arg_list args({
        { { "-c", "--config" }, "config", "path to tracker config file", "", true },
        { { "-i", "--image" }, "image", "path to an image to process", "", false }
    });

    try
    {
        auto p = args.parse(argc, argv);
        string configpath = p["config"];
        auto cfg = json_load(configpath);

        traces_init(cfg);

        if (p.find("image") != p.end())
        {
            p_detector = get_aruco_detector(cfg);
            Mat im = cv::imread(p["image"], 0);
            if (im.empty())
            {
                err_msg("can't read ", p["image"]);
                return -1;
            }

            map<int, polygon_t> markers;
            p_detector->find_markers(im, markers);

            for (auto const& m : markers)
            {
                Vec4f q;
                Vec3f p;
                int id = m.first;
                dbg_msg("marker: ", id, " corners: ", m.second[0], " ", m.second[1], " ", m.second[2], " ", m.second[3], ";");
                p_detector->get_marker_3d_coords(m.second, p, q);
                dbg_msg("markrer ", id, " found: ", p, "; ", q, ";");

                p_detector->draw_frame(im, p, q);
            }

//            p_detector->draw_found_markers(im, markers);
            cv::imshow("1", im);
            cv::waitKey();
            cv::imwrite("debug.png", im);
        }
        else
        {
            traces_init(cfg);
            imgdump_init(cfg);
            run_tracker(cfg);
        }
    }
    catch (invalid_argument& e)
    {
        err_msg(e.what());
        cout << args.help_message() << endl;
        return -1;
    }
    catch (exception& e)
    {
        err_msg(e.what());
        return -1;
    }

    return 0;
}
