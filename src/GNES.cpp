#include "GNES.h"
#include <iostream>

void GNES::Run(const std::string& RomPath)
{
    std::cout << "Hello World" << std::endl;
    std::cout << RomPath << std::endl;

    mCPU = std::make_unique<CPU>(CPU_TIMING::NTSC);
}
