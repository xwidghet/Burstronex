#pragma once
#include "CPU.h"

#include <memory>
#include <string>


class GNES {
    std::unique_ptr<CPU> mCPU;

public:
    void Run(const std::string& RomPath);
};
