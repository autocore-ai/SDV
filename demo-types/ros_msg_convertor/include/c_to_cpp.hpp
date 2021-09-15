#pragma once
#include <common.hpp>

class CToCpp
{
private:
    /* data */
public:
    CToCpp(/* args */);
    ~CToCpp();
    void Time(void *src, void *dst);
    // void Time1(int a, int b);
    void *CreateHeader(int count);
    void *CovHeader(void *data);
};

void *GetCovCToCpp();
