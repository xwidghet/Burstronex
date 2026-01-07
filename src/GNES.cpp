#include "GNES.h"

#include "RomLoader.h"
#include <iostream>

void GNES::Run(const std::string& RomPath)
{
    RomLoader::Load(RomPath);

    mCPU = std::make_unique<CPU>(CPU_TIMING::NTSC);
}
