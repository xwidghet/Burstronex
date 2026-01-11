#include "GNES.h"

#include "RomLoader.h"
#include "RomParameters.h"

#include <chrono>

void GNES::Run(const std::string& RomPath)
{
    auto ROM = RomLoader::Load(RomPath);

    if (ROM.PrgRomMemory.size() == 0)
        return;

    mCPU = std::make_unique<CPU>();
    mCPU->Init(ROM);

    int64_t TotalCycles = 0;
    while (true)
    {
        auto TimeAtInstruction = std::chrono::steady_clock::now();

        auto CyclesUsed = mCPU->ExecuteNextInstruction();

        auto ExecutionTime = mCPU->GetCycleTime() * CyclesUsed;
        auto TimePassed = std::chrono::steady_clock::now() - TimeAtInstruction;

        double DeltaTime = std::chrono::duration<double>(TimePassed).count();
        while (DeltaTime < ExecutionTime)
        {
            TimePassed = std::chrono::steady_clock::now() - TimeAtInstruction;
            DeltaTime = std::chrono::duration<double>(TimePassed).count();
        }

        // Debug Remove me later.
        TotalCycles += CyclesUsed;
        if (TotalCycles > 50)
            break;
    }
}
