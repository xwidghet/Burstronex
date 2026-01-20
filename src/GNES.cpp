#include "GNES.h"

#include "Logger.h"
#include "RomLoader.h"
#include "RomParameters.h"
#include "Timer.h"

#include <thread>

void GNES::Run(const std::string& RomPath)
{
    mLog = std::make_unique<Logger>();

    auto ROM = RomLoader::Load(RomPath);

    if (ROM.PrgRomMemory.size() == 0)
        return;

    mRAM = std::make_unique<MemoryMapper>(ROM.ChrRomMemory, ROM.PrgRomMemory);

    mAPU = std::make_unique<APU>();
    mPPU = std::make_unique<PPU>();
    mCPU = std::make_unique<CPU>();
    mRenderer = std::make_unique<Renderer>();

    mCPU->Init(ROM, &*mRAM, &*mPPU);
    mAPU->Init(&*mRAM, &*mCPU, ROM.CPUTimingMode);
    mPPU->Init(&*mRAM, &*mRenderer, &ROM.ChrRomMemory);

    mRAM->Init(&*mCPU, &*mPPU, &*mRenderer);

    mRenderer->Init(std::bind(&GNES::RequestShutdown, this));
    std::jthread RenderThread(&Renderer::Tick, &*mRenderer);

    Timer ClockTimer;

    Timer EmulatorSpeedTimer;
    mEmulatorSpeed = 1.f;

    int64_t LastCPUCycles = mCPU->GetCycleCount();
    uint8_t WaitCycles = mCPU->GetCycleCount();

    // Execute PPU and APU for boot cycle count
    mPPU->Execute(mCPU->GetCycleCount());
    mAPU->Execute(mCPU->GetCycleCount());

    while (mbIsRunning)
    {
        auto CyclesUsed = mCPU->ExecuteNextInstruction();
        WaitCycles += CyclesUsed;

        mPPU->Execute(CyclesUsed);
        mAPU->Execute(CyclesUsed);

        if (WaitCycles >= 50)
        {
            auto ExecutionTime = mCPU->GetCycleTime() * WaitCycles;
            WaitCycles = 0;

            // Dynamic Rate Control of Emulator Speed via Audio buffer.
            float BufferFillSpeed = (0.5f - mAPU->GetBufferFillPercentage()) / 0.5f;
            ExecutionTime -= ExecutionTime*BufferFillSpeed;

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
          //  break;
    }

    // Headless test rom debug.
    //mLog->Log(ELOGGING_SOURCES::GNES, ELOGGING_MODE::INFO, "0x02: {0}, 0x03: {1}\n", mRAM->Read8Bit(0x02), mRAM->Read8Bit(0x03));
   // mLog->Log(ELOGGING_SOURCES::GNES, ELOGGING_MODE::INFO, "0x02: {0}, 0x03: {1}\n", mRAM->Read8Bit(0x6004), mRAM->Read8Bit(0x6005));
}

void GNES::RequestShutdown()
{
    mbIsRunning = false;
}
