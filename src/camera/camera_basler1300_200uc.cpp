#include "camera.h"

#include <pylon/PylonIncludes.h>
#include <pylon/PylonUtilityIncludes.h>
#include "common.h"
#include "traces.h"


using namespace Pylon;
using namespace std;


class CImageHandler : public CImageEventHandler
{
private:
    image_handler_t  m_handler;

public:
    void set_user_handler(image_handler_t const& handler)
    {
        m_handler = handler;
    }

    virtual void OnImageGrabbed(CInstantCamera& camera, CGrabResultPtr const& ptrGrabResult)
    {
        // dbg_msg("cam callback begins");

        if (ptrGrabResult->GrabSucceeded())
        {
            if (m_handler)
            {
                uint8_t const* data = (uint8_t*) ptrGrabResult->GetBuffer();
                int const Nx = ptrGrabResult->GetWidth();
                int const Ny = ptrGrabResult->GetHeight();
                uint64_t ts = ptrGrabResult->GetTimeStamp() / 1000;
                m_handler(data, Nx, Ny, ts);
            }
        }
        else
        {
            err_msg(ptrGrabResult->GetErrorCode(), " ", ptrGrabResult->GetErrorDescription());
        }

        // dbg_msg("cam callback ends");
    }

    virtual void OnImagesSkipped(CInstantCamera& camera, size_t countOfSkippedImages)
    {
        warn_msg(countOfSkippedImages, " frames skipped");
    }
};

class BaslerCamera : public camera
{
private:
    unique_ptr<CInstantCamera> m_camera;
    unique_ptr<CImageHandler> m_camera_handler;

public:
    BaslerCamera()
    {
        PylonInitialize();

        try
        {
            m_camera.reset(new CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice()));
            m_camera->MaxNumBuffer = 5;
            m_camera->Open();
            m_camera_handler.reset(new CImageHandler);
            m_camera->RegisterImageEventHandler(m_camera_handler.get(), RegistrationMode_Append, Cleanup_None);
        }
        catch (const GenericException &e)
        {
            throw runtime_error(string("camera error: ") + e.GetDescription());
        }    	
    }

    ~BaslerCamera()
    {
        stop();
        m_camera->DeregisterImageEventHandler(m_camera_handler.get());
        m_camera_handler.reset(nullptr);
        m_camera.reset(nullptr);
        PylonTerminate();
    }

    void load(string const& config_filename)
    {
        if (!config_filename.empty())
            CFeaturePersistence::Load(config_filename.c_str(), &m_camera->GetNodeMap(), true);
    }

    void set_handler(image_handler_t const& handler)
    {
        m_camera_handler->set_user_handler(handler);
    }

    void run()
    {
        m_camera->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
    }

    void stop()
    {
        m_camera->StopGrabbing();
    }

    string description()
    {
        return m_camera->GetDeviceInfo().GetModelName().c_str();
    }

    string image_type()
    {
    	return "BayerBGGR";
    }

};

camera* get_camera()
{
	static BaslerCamera cam;
	return &cam;

    // if (!cam)
    //     throw runtime_error("camera wasn't initalized");

    // return cam.get();
}

void init_camera(jsonxx::Object const& jsoncfg)
{
    if (!jsoncfg.has<jsonxx::Object>("basler"))
        throw runtime_error("config file doesn't have 'basler' entry");

    auto const& json_basler = jsoncfg.get<jsonxx::Object>("basler");
    auto const& config_path = json_basler.get<jsonxx::String>("config_path");
    static_cast<BaslerCamera*>(get_camera())->load(config_path);
}
