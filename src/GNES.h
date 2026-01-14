#pragma once

#include "APU.h"
#include "CPU.h"
#include "PPU.h"

#include "MemoryMapper.h"

#include <memory>
#include <string>


class GNES {
    std::unique_ptr<APU> mAPU;
    std::unique_ptr<CPU> mCPU;
    std::unique_ptr<PPU> mPPU;

    std::unique_ptr<MemoryMapper> mRAM;

public:
    void Run(const std::string& RomPath);
};
