#include <c_to_cpp.hpp>

CToCpp::CToCpp(/* args */)
{
    std::cout << "CToCpp" << std::endl;
}

CToCpp::~CToCpp()
{
    std::cout << "~CToCpp" << std::endl;
}

void CToCpp::Time(void *src, void *dst)
{
    std::cout << "CToCpp::Time" << std::endl;
}

// void CToCpp::Time1(int a, int b)
// {
//     std::cout << "CToCpp::Time" << a << ":" << b << std::endl;
// }

void *CToCpp::CreateHeader(int count)
{
    auto stamp = builtin_interfaces::msg::Time{};
    auto ret = std::make_shared<std_msgs::msg::Header>();
    stamp.sec = count;
    ret->frame_id = std::string{};
    ret->stamp = stamp;
    return ret.get();
}

void *CToCpp::CovHeader(void *data)
{
    auto d = (std_msgs__msg__Header *)data;
    std::cout << "Get " << d->stamp.sec << std::endl;
    auto stamp = builtin_interfaces::msg::Time{};
    auto ret = std::make_shared<std_msgs::msg::Header>();
    ret->frame_id = std::string{};
    ret->stamp = stamp;
    return ret.get();
}

static std::shared_ptr<CToCpp> sp;

void *GetCovCToCpp()
{
    sp = std::make_shared<CToCpp>();
    return sp.get();
}
