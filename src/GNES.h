#pragma once

#include "APU.h"
#include "CPU.h"
#include "PPU.h"

#include "MemoryMapper.h"

#include <memory>
#include <string>


class GNES {
    std::unique_ptr<APU> mAPU = nullptr;
    std::unique_ptr<CPU> mCPU = nullptr;
    std::unique_ptr<PPU> mPPU = nullptr;

    std::unique_ptr<MemoryMapper> mRAM = nullptr;

    float mEmulatorSpeed = 0.f;

public:
    void Run(const std::string& RomPath);
};
