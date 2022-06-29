#include <map>
#include <memory>
#include <iostream>
#include "camera.h"


using CameraCreator = std::function<CameraPtr(Json::Value const& json_basler)>;

class CameraFactory
{
    std::map<std::string, CameraCreator> _creators;
    std::weak_ptr<Camera> _camera;

public:
    static CameraFactory& instance()
    {
        static CameraFactory factory;
        return factory;
    }

    void register_camera(std::string const& name, CameraCreator&& creator)
    {
        _creators[name] = std::forward<CameraCreator>(creator);
    }

    CameraPtr init_camera(Json::Value const& cfg)
    {
        if (_camera.use_count() > 0)
            throw_runtime_error("already initialized");

        for (auto& creator : _creators)
        {
            if (json_has(cfg, creator.first))
            {
                auto entry = json_get(cfg, creator.first.c_str());
                auto camera = creator.second(entry);
                _camera = camera;
                return camera;
            }
        }

        throw_runtime_error("can't init camera");
    }
};

CameraPtr init_camera(Json::Value const& cfg)
{
    return CameraFactory::instance().init_camera(cfg);
}

struct RegisterCamera
{
    RegisterCamera(std::string const& name, CameraCreator&& creator)
    {
        CameraFactory::instance().register_camera(name, std::forward<CameraCreator>(creator));
    }
};

#define REGISTER_CAMERA(name, function) \
    CameraPtr function(Json::Value const&); \
    static RegisterCamera __##function(name, function);

#ifdef BUILD_FAKE_CAMERA
REGISTER_CAMERA("fake_camera", create_camera_fake);
#endif

#ifdef BUILD_BASLER_CAMERA
REGISTER_CAMERA("basler_camera", create_camera_basler);
#endif
