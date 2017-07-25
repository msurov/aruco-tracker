#include <cmath>
#include "cv_helpers.h"
#include "traces.h"
#include "common.h"

using namespace std;
using namespace cv;


void BinBox3x3(Mat const& src, Mat& dst)
{
    int const Ny = src.rows;
    int const Nx = src.cols;

    dst.create(src.size(), CV_8U);
    Mat buf(1, Nx, CV_8U);
    uchar* pbuf = buf.ptr<uchar>(0);

    {
        uchar const* psrc1 = src.ptr<uchar>(0);
        uchar const* psrc2 = src.ptr<uchar>(1);

        for (int x = 0; x < Nx; ++x)
            pbuf[x] = 2 * psrc1[x] + psrc2[x];
    }

    for (int y = 0; y < Ny; ++y)
    {
        uchar* pdst = dst.ptr<uchar>(y);

        pdst[0] = pbuf[0] + pbuf[0] + pbuf[1];
        for (int x = 1; x < Nx - 1; ++x)
            pdst[x] = pbuf[x - 1] + pbuf[x + 0] + pbuf[x + 1];
        pdst[0] = pbuf[Nx - 2] + pbuf[Nx - 1] + pbuf[Nx - 1];

        // update buf
        uchar const* psrc0 = src.ptr<uchar>(max(y - 1, 0));
        uchar const* psrc2 = src.ptr<uchar>(min(y + 2, Ny - 1));
        for (int x = 0; x < Nx; ++x)
            pbuf[x] += psrc2[x] - psrc0[x];
    }
}

void BinMedian3x3(Mat const& src, Mat& dst)
{
    int const Ny = src.rows;
    int const Nx = src.cols;

    dst.create(src.size(), CV_8U);
    Mat buf(1, Nx, CV_8U);
    uchar* pbuf = buf.ptr<uchar>(0);

    {
        uchar const* psrc1 = src.ptr<uchar>(0);
        uchar const* psrc2 = src.ptr<uchar>(1);

        for (int x = 0; x < Nx; ++x)
            pbuf[x] = 2 * psrc1[x] + psrc2[x];
    }

    for (int y = 0; y < Ny; ++y)
    {
        uchar* pdst = dst.ptr<uchar>(y);

        pdst[0] = pbuf[0] + pbuf[0] + pbuf[1] > 4;
        for (int x = 1; x < Nx - 1; ++x)
            pdst[x] = pbuf[x - 1] + pbuf[x + 0] + pbuf[x + 1] > 4;
        pdst[Nx-1] = pbuf[Nx - 2] + pbuf[Nx - 1] + pbuf[Nx - 1] > 4;

        // update buf
        uchar const* psrc0 = src.ptr<uchar>(max(y - 1, 0));
        uchar const* psrc2 = src.ptr<uchar>(min(y + 2, Ny - 1));
        for (int x = 0; x < Nx; ++x)
            pbuf[x] += psrc2[x] - psrc0[x];
    }
}

void BinEdgeRect3x3(Mat const& src, Mat& dst)
{
    int const Ny = src.rows;
    int const Nx = src.cols;

    dst.create(src.size(), CV_8U);
    Mat buf(1, Nx, CV_8U);
    uchar* pbuf = buf.ptr<uchar>(0);

    {
        uchar const* psrc1 = src.ptr<uchar>(0);
        uchar const* psrc2 = src.ptr<uchar>(1);

        for (int x = 0; x < Nx; ++x)
            pbuf[x] = 2 * psrc1[x] + psrc2[x];
    }

    for (int y = 0; y < Ny; ++y)
    {
        uchar* pdst = dst.ptr<uchar>(y);
        uchar const* psrc1 = src.ptr<uchar>(y, 0);

        pdst[0] = psrc1[0] && pbuf[0] + pbuf[0] + pbuf[1] < 9;
        for (int x = 1; x < Nx - 1; ++x)
            pdst[x] = psrc1[x] && pbuf[x - 1] + pbuf[x + 0] + pbuf[x + 1] < 9;
        pdst[0] = psrc1[Nx - 1] && pbuf[Nx - 2] + pbuf[Nx - 1] + pbuf[Nx - 1] < 9;

        uchar const* psrc0 = src.ptr<uchar>(max(y - 1, 0));
        uchar const* psrc2 = src.ptr<uchar>(min(y + 2, Ny - 1));
        for (int x = 0; x < Nx; ++x)
            pbuf[x] += psrc2[x] - psrc0[x];
    }
}

void BinEdgeCross3x3(Mat const& src, Mat& dst)
{
    int const Ny = src.rows;
    int const Nx = src.cols;

    for (int y = 0; y < Ny; ++y)
    {
        uchar const* psrc0 = src.ptr<uchar>(max(y - 1, 0));
        uchar const* psrc1 = src.ptr<uchar>(y);
        uchar const* psrc2 = src.ptr<uchar>(min(y + 1, Ny - 1));
        uchar* pdst = dst.ptr<uchar>(y);

        int x = 0;

        pdst[x] = 0;
        ++x;

        for (; x < Nx - 1; ++x)
        {
            uchar v = psrc0[x] & psrc1[x - 1] & psrc1[x + 1] & psrc2[x];
            pdst[x] = ~v & psrc1[x];
        }

        pdst[x] = 0;
    }
}

void DrawPoints(Mat& plot, vector<Point2i> const& points, cv::Scalar const& color)
{
    int Nx = plot.cols;
    int Ny = plot.rows;

    for (auto p : points)
    {
        int x = p.x;
        int y = p.y;
        if (in_diap(x, 0, Nx - 1) && in_diap(y, 0, Ny - 1))
        {
            if (plot.type() == CV_8U)
                plot.at<uchar>(y, x) = (uchar)color(0);
            else if (plot.type() == CV_8UC3)
                plot.at<Vec3b>(y, x) = Vec3b(
                    saturate_cast<uchar>(color(0)), 
                    saturate_cast<uchar>(color(1)),
                    saturate_cast<uchar>(color(2))
                );
        }
    }
}

void DrawCircle(cv::Mat& plot, float cx, float cy, float r, cv::Scalar const& color)
{
    vector<Point2i> pts(int(2*4*r));
    int n = (int)pts.size();

    for (int i = 0; i < n; ++ i)
    {
        int x = int(cx + r * sin(2*_PI*i/n) + 0.5);
        int y = int(cy + r * cos(2*_PI*i/n) + 0.5);
        pts[i] = Point2i(x, y);
    }
    DrawPoints(plot, pts, color);
}

void DrawRect(cv::Mat& plot, cv::Rect const& r, cv::Scalar const& color)
{
    int const n = (r.width + r.height) * 2 + 4;
    vector<Point2i> pts;
    pts.reserve(n);

    for (int x = 0; x < r.width; ++ x)
    {
        int x1 = r.x + x;
        int y1 = r.y;
        int y2 = r.br().y;

        pts.push_back(Point2i(x1, y1));
        pts.push_back(Point2i(x1, y2));
    }

    for (int y = 0; y < r.height; ++ y)
    {
        int y1 = r.y + y;
        int x1 = r.x;
        int x2 = r.br().x;

        pts.push_back(Point2i(x1, y1));
        pts.push_back(Point2i(x2, y1));
    }

    DrawPoints(plot, pts, color);
}

void DrawCross(cv::Mat& plot, int x, int y, int size, cv::Scalar const& color)
{
    vector<Point2i> pts;
    pts.resize(size * 4);

    for (int i = 0; i < size; ++i)
    {
        pts.push_back(Point2i(x + i, y));
        pts.push_back(Point2i(x - i, y));
        pts.push_back(Point2i(x, y + i));
        pts.push_back(Point2i(x, y - i));
    }

    DrawPoints(plot, pts, color);
}

void DrawLine(cv::Mat& plot, cv::Point2f const& p1, cv::Point2f const& p2, cv::Scalar const& color)
{
    Point2f dif = (p2 - p1);
    int maxdif = int(max((float)fabs(dif.x), (float)fabs(dif.y)));

    vector<Point2i> pts;
    pts.resize(maxdif + 1);

    for (int i = 0; i <= maxdif; ++i)
    {
        Point2f p = p1 + (p2 - p1) * (i * 1.0f / max(maxdif, 1));
        pts.push_back(Point2i(lround(p.x), lround(p.y)));
    }

    DrawPoints(plot, pts, color);
}

void CopyChannel(Mat const& src, Mat& dst, int ch)
{
    int const Nx = src.cols;
    int const Ny = src.rows;

    assert(src.type() == CV_8U && dst.type() == CV_8UC3);

    for (int y = 0; y < Ny; ++ y)
    {
        uchar const* psrc = src.ptr<uchar>(y);
        Vec3b* pdst = dst.ptr<Vec3b>(y);

        for (int x = 0; x < Nx; ++ x)
        {
            pdst[x][ch] = psrc[x];
        }
    }
}

Size get_dims(int size)
{
    int kx, ky;

    if (size % 16 == 0 && size % 9 == 0)
    {
        kx = 16;
        ky = 9;
    }
    else if (size % 5 == 0 && size % 4 == 0)
    {
        kx = 5;
        ky = 4;
    }
    else if (size % 4 == 0 && size % 3 == 0)
    {
        kx = 4;
        ky = 3;
    }
    else
    {
    	return Size(0,0);
    }

    int h = (int)sqrt(ky * size / kx);
    int w = kx * h / ky;
    return Size(w, h);
}

Mat read_raw(string const& path)
{
    if (endswith(path, ".raw"))
    {
        ifstream f(path, ios_base::in | ios_base::binary);
        throw_if(f.bad(), runtime_error("can't open " + path));
        f.seekg(0, ios_base::end);
        int len = (int)f.tellg();
        f.seekg(0, ios_base::beg);
        Size s = get_dims(len);
        Mat img(s, CV_8U);

        for (int y = 0; y < img.rows; ++ y)
        {
            uchar* prow = img.ptr<uchar>(y);
            f.read((char*)prow, img.cols);
        }

        return img;
    }
    if (endswith(path, ".bmp") || endswith(path, ".png"))
    {
        Mat im = cv::imread(path);
        return bgr2bggr(im);
    }

    throw runtime_error("invalid image format");
}

Mat bggr2bgr(Mat const& bggr)
{
    Mat bgr;
    // demosaicing(bggr, bgr, CV_BayerRG2BGR);
    cvtColor(bggr, bgr, CV_BayerRG2BGR);
    return bgr;
}

Mat bgr2bggr(Mat const& bgr)
{
    const int Nx = bgr.cols;
    const int Ny = bgr.rows;

    assert(Nx % 2 == 0);
    assert(Ny % 2 == 0);

    Mat res(bgr.size(), CV_8U);

    for (int y = 0; y < Ny; )
    {
        // BG
        Vec3b const* pbgr = bgr.ptr<Vec3b>(y);
        uchar* pres = res.ptr<uchar>(y);

        for (int x = 0; x < Nx;)
        {
            pres[x] = pbgr[x][0];
            ++ x;
            pres[x] = pbgr[x][1];
            ++ x;
        }

        ++ y;

        // GR
        pbgr = bgr.ptr<Vec3b>(y);
        pres = res.ptr<uchar>(y);

        for (int x = 0; x < Nx;)
        {
            pres[x] = pbgr[x][1];
            ++ x;
            pres[x] = pbgr[x][2];
            ++ x;
        }

        ++ y;
    }

    return res;
}
