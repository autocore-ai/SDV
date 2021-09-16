#pragma once
#include <common.hpp>

class CppToC
{
private:
    /* data */
public:
    CppToC(/* args */);
    ~CppToC();
    void *Header(void *);
    void *CreateHeader(int count);
    void *CovHeader();
};

void *GetCovCppToC();
