#include "GNES.h"

#include "Logger.h"
#include "RomLoader.h"
#include "RomParameters.h"
#include "Timer.h"

void GNES::Run(const std::string& RomPath)
{
    mLog = std::make_unique<Logger>();

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

    Timer EmulatorSpeedTimer;
    int64_t LastCPUCycles = mCPU->GetCycleCount();
    uint8_t WaitCycles = 0;

    while (true)
    {
        auto CyclesUsed = mCPU->ExecuteNextInstruction();
        WaitCycles += CyclesUsed;

        mPPU->Execute(CyclesUsed);
        mAPU->Execute(CyclesUsed);

        if (WaitCycles > 50)
        {
            auto ExecutionTime = mCPU->GetCycleTime() * WaitCycles;
            WaitCycles = 0;

            ClockTimer.WaitUntil(ExecutionTime);
            ClockTimer.Reset();
        }

        auto CyclesSinceLastSecond = mCPU->GetCycleCount() - LastCPUCycles;
        if (CyclesSinceLastSecond >= mCPU->GetClockFrequency())
        {
            // Since I only wait every 50+ cycles, and I also emulate whole instructions,
            // there will almost always be more cycles executed than the clock frequency.
            //
            // So let's account for that and normalize the time for a more sane Emulator Speed statistic.
            auto Overshoot = (CyclesSinceLastSecond - mCPU->GetClockFrequency()) / mCPU->GetClockFrequency();

            mEmulatorSpeed = (1.0 / EmulatorSpeedTimer.GetDeltaTime()) - Overshoot;
            mLog->Log(ELOGGING_SOURCES::GNES, ELOGGING_MODE::INFO, "Emulator Speed: {0:.8f}\n", mEmulatorSpeed);

            LastCPUCycles = mCPU->GetCycleCount();
            EmulatorSpeedTimer.Reset();
        }

        // Debug Remove me later.
        //if (mCPU->GetCycleCount() > 26554)
         //   break;
    }

    // Headless test rom debug.
    //mLog->Log(ELOGGING_SOURCES::GNES, ELOGGING_MODE::INFO, "0x02: {0}, 0x03: {1}\n", mRAM->Read8Bit(0x02), mRAM->Read8Bit(0x03));
}
