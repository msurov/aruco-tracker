#include "camera.h"

#include <pylon/PylonIncludes.h>
#include <pylon/PylonUtilityIncludes.h>
#include <cppmisc/traces.h>
#include <cppmisc/timing.h>
#include <cppmisc/files.h>


using namespace Pylon;
using namespace std;


class EventHandler : public CImageEventHandler
{
private:
    ImageHandler  m_handler;
    Camera* m_camera;
    bool _stopreqested;
    bool _executing;
    int64_t _time_sync;

public:
    EventHandler(Camera* camera) : m_camera(camera), 
        _stopreqested(false), _executing(false), _time_sync(-1) {}
    ~EventHandler() { m_camera = nullptr; }

    void set_user_handler(ImageHandler const& handler)
    {
        m_handler = handler;
    }

    bool wait_execution_done(int64_t max_wait_usec)
    {
        _stopreqested = true;
        int64_t t = epoch_usec();
        int64_t wait_until = max_wait_usec <= 0 ? std::numeric_limits<int64_t>::max() : 
            max_wait_usec + t;

        while (_executing && epoch_usec() < wait_until)
        {
            sleep_sec(0.1);
        }

        return !_executing;
    }

    void OnImageGrabbed(CInstantCamera& camera, CGrabResultPtr const& grabres) override
    {
        if (_stopreqested)
            return;
        _executing = true;

        try
        {
            if (grabres->GrabSucceeded())
            {
                int64_t ts = (int64_t)grabres->GetTimeStamp() / 1000;
                if (_time_sync == -1)
                    _time_sync = epoch_usec() - ts;
                ts += _time_sync;

                if (m_handler)
                {
                    uint8_t const* data = (uint8_t*) grabres->GetBuffer();
                    int const Nx = grabres->GetWidth();
                    int const Ny = grabres->GetHeight();
                    RawImageWrap image {
                        data, Nx, Ny, m_camera->image_type(), ts
                    };
                    m_handler(image);
                }
            }
            else
            {
                err_msg(grabres->GetErrorCode(), " ", grabres->GetErrorDescription());
            }
        }
        catch (std::exception const& e)
        {
            err_msg("image handling failed: ", e.what(), ". Stopping.");
            m_camera->stop();
        }
        catch (...)
        {
            err_msg("image handling failed: undefined. Stopping.");
            m_camera->stop();
        }

        // TODO: be sure handler takes less than 1/freq
        _executing = false;
    }

    void OnImagesSkipped(CInstantCamera& camera, size_t nskipped) override
    {
        warn_msg(nskipped, " frames skipped");
    }
};

class BaslerCamera : public Camera
{
private:
    unique_ptr<CInstantCamera> m_camera;
    unique_ptr<EventHandler> m_camera_handler;
    string m_camera_description;

public:
    BaslerCamera()
    {
        PylonInitialize();

        try
        {
            m_camera.reset(new CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice()));
            m_camera->MaxNumBuffer = 5;
            m_camera->Open();
            m_camera_description = m_camera->GetDeviceInfo().GetModelName();
            m_camera_handler.reset(new EventHandler(this));
            m_camera->RegisterImageEventHandler(m_camera_handler.get(), RegistrationMode_Append, Cleanup_None);
        }
        catch (const GenericException &e)
        {
            throw_runtime_error("camera error: ", e.GetDescription());
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
        if (config_filename.empty())
            return;

        if (!file_exists(config_filename))
            throw_runtime_error("can't find ", config_filename);
        
        try
        {
            CFeaturePersistence::Load(config_filename.c_str(), &m_camera->GetNodeMap(), true);
        }
        catch (Pylon::RuntimeException const& e)
        {
            throw_runtime_error(e.what());
        }
        catch (Pylon::InvalidArgumentException const& e)
        {
            throw_invalid_argument(e.what());
        }
        catch (Pylon::GenericException const& e)
        {
            throw_runtime_error(e.what());
        }
    }

    void set_handler(ImageHandler const& handler) override
    {
        m_camera_handler->set_user_handler(handler);
    }

    void run() override
    {
        m_camera->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
    }

    void stop() override
    {
        m_camera->StopGrabbing();
        m_camera_handler->wait_execution_done(sec_to_usec(5));
    }

    char const* description() override
    {
        return m_camera_description.c_str();
    }

    char const* image_type() override
    {
    	// return "BayerBGGR";
        return "gray";
    }

};

CameraPtr create_camera_basler(Json::Value const& json_basler)
{
    auto const& config_path = json_get<std::string>(json_basler, "config_path");
    auto result = std::make_shared<BaslerCamera>();
    result->load(config_path);
    return result;
}
