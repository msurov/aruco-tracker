#include <memory>
#include "publisher.h"
#include <networking/tcp.h>
#include <cppmisc/json.h>
#include <cppmisc/traces.h>


class TCPJsonPublisher : public Publisher
{
private:
    TCPSrv _server;
    ConnectionPtr _connection;
    int _port;

public:
    TCPJsonPublisher(int port);
    ~TCPJsonPublisher();
    bool start() override;
    void term() override;
    bool publish(int64_t t, std::vector<MarkerPose> const& marker_pose_arr) override;
    static PublisherPtr create(Json::Value const& config);
};


static const char __markerpose_templ[] = 
R"({"id": "%d", "p": [%e, %e, %e], "r": [%e, %e, %e], "pr_cov": [%e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e]})";

static const int __markerpose_bufsz = sizeof(__markerpose_templ) + 4 + (3 + 3 + 21) * 16;

inline int to_jsonstr(MarkerPose const& marker_pose, char* buf, int buf_sz)
{
    int id = marker_pose.id;

    double rx = marker_pose.pose.r(0);
    double ry = marker_pose.pose.r(1);
    double rz = marker_pose.pose.r(2);

    double px = marker_pose.pose.p(0);
    double py = marker_pose.pose.p(1);
    double pz = marker_pose.pose.p(2);

    double rx_rx = marker_pose.pose.cov(0,0);
    double rx_ry = marker_pose.pose.cov(0,1);
    double rx_rz = marker_pose.pose.cov(0,2);
    double rx_px = marker_pose.pose.cov(0,3);
    double rx_py = marker_pose.pose.cov(0,4);
    double rx_pz = marker_pose.pose.cov(0,5);

    double ry_ry = marker_pose.pose.cov(1,1);
    double ry_rz = marker_pose.pose.cov(1,2);
    double ry_px = marker_pose.pose.cov(1,3);
    double ry_py = marker_pose.pose.cov(1,4);
    double ry_pz = marker_pose.pose.cov(1,5);

    double rz_rz = marker_pose.pose.cov(2,2);
    double rz_px = marker_pose.pose.cov(2,3);
    double rz_py = marker_pose.pose.cov(2,4);
    double rz_pz = marker_pose.pose.cov(2,5);

    double px_px = marker_pose.pose.cov(3,3);
    double px_py = marker_pose.pose.cov(3,4);
    double px_pz = marker_pose.pose.cov(3,5);

    double py_py = marker_pose.pose.cov(4,4);
    double py_pz = marker_pose.pose.cov(4,5);

    double pz_pz = marker_pose.pose.cov(5,5);

    return snprintf(
        buf, buf_sz,
        __markerpose_templ, 
        id,
        px, py, pz,
        rx, ry, rz,
        px_px, px_py, px_pz, rx_px, ry_px, rz_px, 
               py_py, py_pz, rx_py, ry_py, rz_py, 
                      pz_pz, rx_pz, ry_pz, rz_pz, 
                             rx_rx, rx_ry, rx_rz, 
                                    ry_ry, ry_rz, 
                                           rz_rz
    );
}

TCPJsonPublisher::TCPJsonPublisher(int port) : _server(port), _port(port)
{
}

TCPJsonPublisher::~TCPJsonPublisher()
{
}

bool TCPJsonPublisher::start()
{
    info_msg("waiting for incoming connection on port ", _port);
    _connection = nullptr;
    _connection = _server.wait_for_connection();
    if (!_connection)
    {
        err_msg("Listening to port ", _port, " failed");
        return false;
    }
    return true;
}

void TCPJsonPublisher::term()
{
    _server.stop();
}

bool to_jsonstr(int64_t ts, std::vector<MarkerPose> const& marker_pose_arr, std::string& s)
{
    const int n = marker_pose_arr.size();
    s.clear();
    s.reserve(__markerpose_bufsz * n);

    s = "{\"ts\": " + std::to_string(ts) + ", ";
    s += "\"objects\": [";

    for (int i = 0; i < n; ++ i)
    {
        char buf[__markerpose_bufsz];
        int len = to_jsonstr(marker_pose_arr[i], buf, sizeof(buf));
        if (len <= 0)
        {
            err_msg("internal error: failed to pack object pose into json");
            return false;
        }
        s.append(buf, len);
        if (i != n - 1)
            s += ", ";
    }
    s += "]}";
    return true;
}

bool TCPJsonPublisher::publish(int64_t ts, std::vector<MarkerPose> const& marker_pose_arr)
{
    std::string s;
    bool ok = to_jsonstr(ts, marker_pose_arr, s);
    if (!ok)
    {
        _connection = nullptr;
        throw_runtime_error("internal error: failed to pack object pose into json");
    }

    ok = _connection->write(s.c_str(), s.size() + 1);
    if (!ok)
    {
        _connection = nullptr;
        err_msg("can't send, connection closed by client?");
        return false;
    }
    return true;
}

PublisherPtr create_publisher_jsontcp(Json::Value const& cfg)
{
    int port = json_get<int>(cfg, "port");
    auto ptr = std::make_shared<TCPJsonPublisher>(port);
    return std::static_pointer_cast<Publisher>(ptr);
}
