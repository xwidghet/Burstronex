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

    while (true)
    {
        ClockTimer.Reset();

        auto CyclesUsed = mCPU->ExecuteNextInstruction();
        auto ExecutionTime = mCPU->GetCycleTime() * CyclesUsed;

        mPPU->Execute(CyclesUsed);
        mAPU->Execute(CyclesUsed);

        while (ClockTimer.PeakDeltaTime() < ExecutionTime)
        {
        }

        // Debug Remove me later.
        //if (mCPU->GetCycleCount() > 36554)
         //   break;
    }

    // Headless test rom debug.
    //mLog->Log(ELOGGING_SOURCES::GNES, ELOGGING_MODE::INFO, "0x02: {0}, 0x03: {1}\n", mRAM->Read8Bit(0x02), mRAM->Read8Bit(0x03));
}
