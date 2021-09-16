#include <c_to_cpp.hpp>

CToCpp::CToCpp(/* args */)
{
    std::cout << "CToCpp" << std::endl;
}

CToCpp::~CToCpp()
{
    std::cout << "~CToCpp" << std::endl;
}

void *CToCpp::Header(void *data)
{
    auto ret = std::make_shared<std_msgs::msg::Header>();
    auto ptr = (std_msgs__msg__Header *)data;
    ret->stamp = Get(ptr->stamp);
    return ret.get();
}

void CToCpp::SetValue(void *data)
{
    auto ptr = (std_msgs__msg__Header *)data;
    std::cout << ptr->stamp.sec << std::endl;
    ptr->stamp.sec = 233;
}

void CToCpp::PrintC(void *data)
{
    auto ptr = (std_msgs__msg__Header *)data;
    std::cout << "print:::::::::" << ptr->stamp.sec << std::endl;
}

builtin_interfaces::msg::Time CToCpp::operator=(builtin_interfaces__msg__Time data)
{
    return Get(data);
}

builtin_interfaces::msg::Time CToCpp::Get(builtin_interfaces__msg__Time data)
{
    auto ret = builtin_interfaces::msg::Time{};
    ret.sec = data.sec;
    ret.nanosec = data.nanosec;
    return ret;
}

std::string CToCpp::Get(rosidl_runtime_c__String data)
{
    std::string ret(data.data);
    return ret;
}

std_msgs::msg::Header CToCpp::Get(std_msgs__msg__Header data)
{
    auto ret = std_msgs::msg::Header{};
    ret.frame_id = Get(data.frame_id);
    ret.stamp = Get(data.stamp);
    return ret;
}

std::shared_ptr<std_msgs::msg::Header> CToCpp::Get(std::shared_ptr<std_msgs__msg__Header> data)
{
    auto ret = std::make_shared<std_msgs::msg::Header>();
    ret->frame_id = Get(data->frame_id);
    ret->stamp = Get(data->stamp);
    return ret;
}

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
