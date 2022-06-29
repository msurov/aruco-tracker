#include <map>
#include "publisher.h"


using PublisherCreator = std::function<PublisherPtr(Json::Value const& json_basler)>;

class PublisherFactory
{
    std::map<std::string, PublisherCreator> _creators;
    std::weak_ptr<Publisher> _publisher;

public:

    static PublisherFactory& instance()
    {
        static PublisherFactory factory;
        return factory;
    }

    void register_publisher(std::string const& name, PublisherCreator&& creator)
    {
        _creators[name] = std::forward<PublisherCreator>(creator);
    }

    PublisherPtr init_publisher(Json::Value const& cfg)
    {
        if (_publisher.use_count() > 0)
            throw_runtime_error("already initialized");

        for (auto& creator : _creators)
        {
            if (json_has(cfg, creator.first))
            {
                auto entry = json_get(cfg, creator.first.c_str());
                auto pub = creator.second(entry);
                _publisher = pub;
                return pub;
            }
        }

        throw_runtime_error("can't init publisher");
    }
};

struct RegisterPublisher
{
    RegisterPublisher(std::string const& name, PublisherCreator&& creator)
    {
        PublisherFactory::instance().register_publisher(name, std::forward<PublisherCreator>(creator));
    }
};

#define REGISTER_PUBLISHER(name, function) \
    PublisherPtr function(Json::Value const&); \
    static RegisterPublisher __##function(name, function);

REGISTER_PUBLISHER("publish_file", create_publisher_file);
REGISTER_PUBLISHER("publish_tcpjson", create_publisher_jsontcp);

PublisherPtr create_publisher(Json::Value const& cfg)
{
    return PublisherFactory::instance().init_publisher(cfg);
}
