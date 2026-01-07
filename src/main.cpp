#include "GNES.h"

#include <string>

int main(int argc, char* argv[])
{
    std::string RomPath = "/InavlidRomPath/LOL.nes";
    if (argc > 1)
    {
        RomPath = std::string(argv[1]);
    }

    GNES Emulator;
    Emulator.Run(RomPath);

}
