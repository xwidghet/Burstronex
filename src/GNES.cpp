#include "GNES.h"

#include "RomLoader.h"
#include "RomParameters.h"

void GNES::Run(const std::string& RomPath)
{
    auto ROM = RomLoader::Load(RomPath);

    if (ROM.PrgRomMemory.size() == 0)
        return;

    mCPU = std::make_unique<CPU>(ECPU_TIMING::NTSC);
}
