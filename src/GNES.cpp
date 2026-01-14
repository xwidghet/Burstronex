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

    mPPU = std::make_unique<PPU>(&*mRAM);
    mPPU->Init(&ROM.ChrRomMemory);

    mCPU = std::make_unique<CPU>();
    mCPU->Init(ROM, &*mRAM, &*mPPU);

    Timer ClockTimer;

    while (true)
    {
        ClockTimer.Reset();

        // PPU ticks 3x as fast as the CPU. For now, tick 3 times for simplicity.
        mPPU->ExecuteCycle();
        mPPU->ExecuteCycle();
        mPPU->ExecuteCycle();

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
