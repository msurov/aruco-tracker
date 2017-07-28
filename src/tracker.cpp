#include "traces.h"
#include "camera.h"
#include "parse_args.h"
#include "common.h"
#include "imgdump.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include "aruco_detector.h"

using namespace std;
using namespace cv;


static bool brun = false;
static unique_ptr<ArucoDetector> p_detector;
const string node_name = "marker_tracker";
const string pub_name = "/basler/marker";
unique_ptr<ros::NodeHandle> node;
ros::Publisher pub;

static void camera_handler(uint8_t const* data, int Nx, int Ny, int64_t frame_timestamp)
{
    if (!brun)
        return;

    try
    {
        if (p_detector)
        {
            Mat im(Ny, Nx, CV_8U, (void*)data);
            map<int, marker_t> markers;
            p_detector->detect(im, markers);

            auto m_origin = markers.find(38);
            auto m_obj = markers.find(39);

            if (m_origin != markers.end() && m_obj != markers.end())
            {
                Matx44f T_origin = m_origin->second.T;
                Matx44f T_obj = m_obj->second.T;
                Matx44f T = T_origin.inv() * T_obj;

                geometry_msgs::Pose pose;
                pose.position.x = T(0,3);
                pose.position.y = T(1,3);
                pose.position.z = T(2,3);
                pub.publish(pose);
                dbg_msg("found: ", pose.position.x, " ", pose.position.y, " ", pose.position.z);
            }
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

        if (p.find("image") != p.end())
        {
            p_detector = get_aruco_detector(cfg);
            Mat im = cv::imread(p["image"], 0);
            map<int, marker_t> markers;
            p_detector->detect(im, markers);
        }
        else
        {
            ros::init(argc, argv, node_name);
            node.reset(new ros::NodeHandle());
            pub = node->advertise<geometry_msgs::Pose>(pub_name, 10);

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
