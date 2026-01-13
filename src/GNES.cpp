#include "GNES.h"

#include "RomLoader.h"
#include "RomParameters.h"

#include <chrono>

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

    while (true)
    {
        // PPU ticks 3x as fast as the CPU. For now, tick 3 times for simplicity.
        mPPU->ExecuteCycle();
        mPPU->ExecuteCycle();
        mPPU->ExecuteCycle();

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
        if (mCPU->GetCycleCount() > 36554)
            break;
    }

    std::cout << std::format("0x02: {0}, 0x03: {1}", mRAM->Read8Bit(0x02), mRAM->Read8Bit(0x03)) << std::endl;
}
