#include <cpp_to_c.hpp>

CppToC::CppToC(/* args */)
{
    std::cout << "CppToC" << std::endl;
}

CppToC::~CppToC()
{
    std::cout << "~CppToC" << std::endl;
}

void* CppToC::Header(void *data)
{
    auto sp = std::shared_ptr<std_msgs::msg::Header>((std_msgs::msg::Header *)data);
    std::cout << "CppToC::Header" << sp->stamp.sec << std::endl;
    return nullptr;
}

void *CppToC::CreateHeader(int count)
{
    auto stamp = builtin_interfaces__msg__Time{};
    stamp.sec = count;
    auto ret = std::make_shared<std_msgs__msg__Header>();
    ret->frame_id = rosidl_runtime_c__String{};
    ret->stamp = stamp;
    std::cout << "Set " << ret->stamp.sec << std::endl;
    return ret.get();
}

void *CppToC::CovHeader()
{
    auto stamp = builtin_interfaces__msg__Time{};
    stamp.sec = 233;
    auto ret = std::make_shared<std_msgs__msg__Header>();
    ret->frame_id = rosidl_runtime_c__String{};
    ret->stamp = stamp;
    return ret.get();
}

static std::shared_ptr<CppToC> sp;

void *GetCovCppToC()
{
    sp = std::make_shared<CppToC>();
    return sp.get();
}
