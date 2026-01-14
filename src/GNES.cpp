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

        if (EmulatorSpeedTimer.PeakDeltaTime() >= 1.0)
        {
            int64_t CurrentCPUCycles = mCPU->GetCycleCount();
            int64_t Delta = CurrentCPUCycles - LastCPUCycles;

            mEmulatorSpeed = double(Delta) / mCPU->GetClockFrequency();
            mLog->Log(ELOGGING_SOURCES::GNES, ELOGGING_MODE::INFO, "Emulator Speed: {0:.8f}\n", mEmulatorSpeed);

            LastCPUCycles = CurrentCPUCycles;
            EmulatorSpeedTimer.Reset();
        }

        // Debug Remove me later.
        //if (mCPU->GetCycleCount() > 36554)
         //   break;
    }

    // Headless test rom debug.
    //mLog->Log(ELOGGING_SOURCES::GNES, ELOGGING_MODE::INFO, "0x02: {0}, 0x03: {1}\n", mRAM->Read8Bit(0x02), mRAM->Read8Bit(0x03));
}
