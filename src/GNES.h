#pragma once

#include "MemoryMapper.h"

#include "APU.h"
#include "PPU.h"
#include "CPU.h"
#include "Renderer.h"
#include "StatisticsManager.h"

#include <memory>
#include <string>

class GNES {
    std::unique_ptr<APU> mAPU = nullptr;
    std::unique_ptr<CPU> mCPU = nullptr;
    std::unique_ptr<PPU> mPPU = nullptr;

    std::unique_ptr<MemoryMapper> mRAM = nullptr;

    std::unique_ptr<Renderer> mRenderer = nullptr;

    bool mbIsRunning = true;
    bool mbThrottle = true;

    EmulationStatistics mEmulationStatistics;

public:
    void Run(const std::string& RomPath);

    void RequestShutdown();

    void ToggleThrottle();
};
