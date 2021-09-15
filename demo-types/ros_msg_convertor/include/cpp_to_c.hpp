#pragma once
#include <common.hpp>

class CppToC
{
private:
    /* data */
public:
    CppToC(/* args */);
    ~CppToC();
    void Time(void *src, void *dst);
    void *CreateHeader(int count);
    void *CovHeader();
};

void *GetCovCppToC();
