#include "MemoryMapper.h"

#include "APU.h"
#include "PPU.h"
#include "OpCodeDecoder.h"

#include <cstring>

MemoryMapper::MemoryMapper(const std::vector<char>& ChrRomMemory, const std::vector<char>& PrgRomMemory)
{
    // Supposedly I should make this a 2D array...I'll do that later when I learn why
    mMemory = std::vector<uint8_t>(65536);

    mChrRomMemory = ChrRomMemory;
    mPrgRomMemory = PrgRomMemory;

    // What to do with trainer memory, do we need to increase size to load it?

    mPrgRomLocation =  0x8000;
    std::memcpy(mMemory.data() + mPrgRomLocation, PrgRomMemory.data(), PrgRomMemory.size());

    // Chr Memory requires a mapper to dynamically load information into 0x0000 -> 0x1FFF range during rendering.
    // Since it's only used for rendering, I can skip it for now.
    mChrRomLocation = 0x0000;
}

void MemoryMapper::Init(PPU* PPU)
{
    mPPU = PPU;
}

uint32_t MemoryMapper::MapAddress(uint32_t Address)
{
    // Internal RAM, 0x0000 - 0x07FF, Mirrored up to 0x1FFF
    if (Address <= 0x1FFF)
        Address = 0x0000 + (Address - 0x0000) % (0x07FF + 1);
    // NES PPU Registers, 0x2000 - 0x2007, Mirrored up to 0x3FFF
    else if (Address <= 0x3FFF)
        Address = 0x2000 + (Address - 0x2000) % ((0x2007 - 0x2000) + 1);
    // Prg ROM Area, with iNES 2.0 ROMS it could be as small as 8KB X 4 Mirrors, but generally 16KB X 2 Mirrors or 32KB No Mirrors
    else if (Address >= 0x8000)
        Address = 0x8000 + (Address - 0x8000) % (mPrgRomMemory.size());

    return Address;
}

uint8_t MemoryMapper::Read8Bit(const uint32_t Address)
{
    uint8_t Value = mMemory[MapAddress(Address)];

    if (Address == STATUS_ADDRESS)
    {
        mMemory[MapAddress(Address)] = Value & (~static_cast<uint8_t>(ESTATUS_READ_MASKS::FRAME_INTERRUPT));
    }
    else if (Address == PPUSTATUS_ADDRESS)
    {
        mMemory[MapAddress(Address)] = Value & (~static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG));
        mPPU->ClearWRegister();
    }

    return Value;
}

void MemoryMapper::Write8Bit(const uint32_t Address, uint8_t Value)
{
    // Cannot write to Program Read Only Memory (ROM)
    if (Address >= mPrgRomLocation)
        return;

    if (Address == PPUADDR_ADDRESS || Address == PPUSCROLL_ADDRESS)
    {
        mMemory[MapAddress(Address)] = Value & (~static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG));
        mPPU->ToggleWRegister();
    }

    mMemory[MapAddress(Address)] = Value;
}

uint16_t MemoryMapper::Read16Bit(const uint32_t Address)
{
    uint16_t Low = mMemory[MapAddress(Address)];
    uint16_t High = mMemory[MapAddress(Address+1)];

    return uint16_t((High << 8) | Low);
}

void MemoryMapper::Write16Bit(const uint32_t Address, uint16_t Value)
{
    // Cannot write to Program Read Only Memory (ROM)
    if (Address >= mPrgRomLocation)
        return;

    uint16_t Low = Value & 0xFF;
    uint16_t High = (Value >> 8) & 0xFF;

    // What happens when it overflows?
    mMemory[MapAddress(Address)] = Low;
    mMemory[MapAddress(Address+1)] = High;
}

uint8_t MemoryMapper::ReadRegister(const uint32_t Address)
{
    return mMemory[MapAddress(Address)];
}

void MemoryMapper::WriteRegister(const uint32_t Address, uint8_t Value)
{
    // Cannot write to Program Read Only Memory (ROM)
    if (Address >= mPrgRomLocation)
        return;

    mMemory[MapAddress(Address)] = Value;
}


const uint16_t MemoryMapper::GetPrgRomLocation() const
{
    return mPrgRomLocation;
}

const uint16_t MemoryMapper::GetChrRomLocation() const
{
    return mChrRomLocation;
}
