#include <map>
#include <stdexcept>
#include <opencv2/aruco.hpp>


template <typename T>
T inline sign(T const& a)
{
    return a > static_cast<T>(0) ? static_cast<T>(1) : static_cast<T>(-1);
} 

struct marker_t
{
    cv::Matx44f T;
    cv::Point2f c1, c2, c3, c4;

    inline cv::Point3f pos() const
    {
        cv::Point3f res(T(0,3), T(1,3), T(2,3));
        return res;
    }

    inline cv::Matx33f R() const
    {
        cv::Matx33f R(
             T(0,0), T(0,1), T(0,2),
             T(1,0), T(1,1), T(1,2),
             T(2,0), T(2,1), T(2,2)
        );
        return R;
    }
};

class ArucoDetector
{
private:
    cv::Ptr<cv::aruco::Dictionary> m_dict;
    float m_side;
    cv::Matx33f m_K;
    std::vector<float> m_distortion;
    std::vector<cv::Point2f> m_aligned_quad;

public:
    ArucoDetector(
        int nrows, float side, cv::Matx33f const& K, 
        std::vector<float> const& distortion
        )
    {
        int v = 
            nrows == 4 ? cv::aruco::DICT_4X4_250 : 
            nrows == 5 ? cv::aruco::DICT_5X5_250 : 
            nrows == 6 ? cv::aruco::DICT_6X6_250 : -1; 

        if (v == -1)
            throw std::runtime_error("unknown argument");

        m_dict = cv::aruco::getPredefinedDictionary(v);
        m_side = side;
        m_K = K;
        m_distortion = distortion;
        m_aligned_quad.push_back(cv::Point2f(0, side));
        m_aligned_quad.push_back(cv::Point2f(side, side));
        m_aligned_quad.push_back(cv::Point2f(side, 0));
        m_aligned_quad.push_back(cv::Point2f(0, 0));
    }

    void detect(cv::Mat const& im, std::map<int, marker_t>& markers)
    {
        std::vector<std::vector<cv::Point2f>> quads;
        std::vector<int> ids;
        cv::aruco::detectMarkers(im, m_dict, quads, ids);

        // TODO cornersubpix

        for (int i = 0; i < (int)quads.size(); ++ i)
        {
            auto const& q = quads[i];
            int id = ids[i];

            std::vector<cv::Point2f> q_normed(q.size());

            // v1
            cv::undistortPoints(q, q_normed, m_K, m_distortion, cv::noArray(), cv::Matx33d::eye());
            cv::Matx33f H = cv::findHomography(m_aligned_quad, q_normed);
            auto c = H.col(0);
            float k = sqrtf(c.dot(c));
            cv::Matx33f P = H * (-1.f / k) * sign(cv::determinant(H));

            cv::Point3f ex = cv::Point3f(P(0,0), P(1,0), P(2,0));
            cv::Point3f ey = cv::Point3f(P(0,1), P(1,1), P(2,1));
            cv::Point3f ez = ex.cross(ey);

            cv::Matx44f T = cv::Matx44f(
                ex.x, ey.x, ez.x, P(0,2),
                ex.y, ey.y, ez.y, P(1,2),
                ex.z, ey.z, ez.z, P(2,2),
                   0,    0,    0,      1
            );
            markers[id].T = T;
            markers[id].c1 = q[0];
            markers[id].c2 = q[1];
            markers[id].c3 = q[2];
            markers[id].c4 = q[3];
        }
    }
};

template <int Ny, int Nx>
cv::Matx<double, Ny, Nx> json_get_matx(jsonxx::Object const& cfg, std::string const& name)
{
    auto json_A = json_get_vector<double>(cfg, name);
    cv::Matx33d A;

    if (json_A.size() != Nx * Ny)
        throw std::runtime_error("expected array " + name + " len is " + std::to_string(Nx * Ny));

    for (int y = 0; y < Ny; ++y)
    {
        for (int x = 0; x < Nx; ++x)
        {
            A(y, x) = json_A[x + Nx * y];
        }
    }

    return A;
}

std::unique_ptr<ArucoDetector> get_aruco_detector(jsonxx::Object const& cfg)
{
    auto cam = json_get<jsonxx::Object>(cfg, "intrinsics");
    auto K = json_get_matx<3, 3>(cam, "K");
    auto distortion = json_get_vector<float>(cam, "distortion");

    auto marker = json_get<jsonxx::Object>(cfg, "marker");
    int nrows = json_get<jsonxx::Number>(marker, "nrows");
    float side = json_get<jsonxx::Number>(marker, "side");

    return std::unique_ptr<ArucoDetector>(new ArucoDetector(nrows, side, K, distortion));
}
