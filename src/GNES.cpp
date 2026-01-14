#include "GNES.h"

#include "RomLoader.h"
#include "RomParameters.h"
#include "Timer.h"

#include <iostream>
#include <format>

void GNES::Run(const std::string& RomPath)
{
    auto ROM = RomLoader::Load(RomPath);

    if (ROM.PrgRomMemory.size() == 0)
        return;

    mRAM = std::make_unique<MemoryMapper>(ROM.ChrRomMemory, ROM.PrgRomMemory);

    mAPU = std::make_unique<APU>();
    mAPU->Init(&*mRAM);

    mPPU = std::make_unique<PPU>();
    mPPU->Init(&*mRAM, &ROM.ChrRomMemory);

    mCPU = std::make_unique<CPU>();
    mCPU->Init(ROM, &*mRAM, &*mPPU);

    Timer ClockTimer;

    while (true)
    {
        ClockTimer.Reset();

        mPPU->Execute();
        mAPU->Execute();

        auto CyclesUsed = mCPU->ExecuteNextInstruction();
        auto ExecutionTime = mCPU->GetCycleTime() * CyclesUsed;

        while (ClockTimer.PeakDeltaTime() < ExecutionTime)
        {
        }

        // Debug Remove me later.
        //if (mCPU->GetCycleCount() > 36554)
         //   break;
    }

    // Headless test rom debug.
    //std::cout << std::format("0x02: {0}, 0x03: {1}", mRAM->Read8Bit(0x02), mRAM->Read8Bit(0x03)) << std::endl;
}
