#include <api.hpp>


Api::Api()
{
    std::cout << "Api" << std::endl;
}

Api::~Api()
{
    std::cout << "~Api" << std::endl;
}

static std::shared_ptr<Api> sp;

void *GetApi()
{
    sp = std::make_shared<Api>();
    return sp.get();
}
