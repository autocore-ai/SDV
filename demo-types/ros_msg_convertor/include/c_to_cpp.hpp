#pragma once
#include <common.hpp>

class CToCpp
{
private:
    /* data */
public:
    CToCpp(/* args */);

    ~CToCpp();
    void *Header(void *);
    void *CreateHeader(int count);
    void *CovHeader(void *data);
    void SetValue(void *data);
    void PrintC(void *data);

private:
    void *tmp;
    builtin_interfaces::msg::Time Get(builtin_interfaces__msg__Time data);
    std::string Get(rosidl_runtime_c__String data);
    std_msgs::msg::Header Get(std_msgs__msg__Header data);
    std::shared_ptr<std_msgs::msg::Header> Get(std::shared_ptr<std_msgs__msg__Header> data);
    builtin_interfaces::msg::Time operator=(builtin_interfaces__msg__Time);
};

void *GetCovCToCpp();
