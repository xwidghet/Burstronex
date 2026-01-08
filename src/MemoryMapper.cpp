#include "MemoryMapper.h"
#include "OpCodeDecoder.h"

#include <cstring>

MemoryMapper::MemoryMapper(const std::vector<char>& ChrRomMemory, const std::vector<char>& PrgRomMemory)
{
    // Supposedly I should make this a 2D array...I'll do that later when I learn why
    mMemory = std::vector<uint8_t>(65536);

    // What to do with trainer memory, do we need to increase size to load it?

    // PRG code starts at 0x8000 and goes to 0xFFFF
    // Seems like it should be always shifted to the end of the memory space, since the reset vector is at the very end of the address range.
    std::memcpy(mMemory.data() + 0xFFFF - PrgRomMemory.size(), PrgRomMemory.data(), PrgRomMemory.size());

    // Chr Memory requires a mapper to dynamically load information into 0x0000 -> 0x1FFF range during rendering.
    // Since it's only used for rendering, I can skip it for now.

}

uint8_t MemoryMapper::Read8Bit(const int32_t Address)
{
    // Todo:: Implement memory mapping, wraparound, address sanitization, etc.
    // Implement Little-indean -> Big-Indean. Either always execute it, or convert rom to x86 endianess.
    return mMemory[Address];
}

void MemoryMapper::Write8Bit(const int32_t Address, uint8_t Value)
{
    mMemory[Address] = Value;
}

uint16_t MemoryMapper::Read16Bit(const int32_t Address)
{
    // Todo:: Implement memory mapping, wraparound, address sanitization, etc.
    // Implement Little-indean -> Big-Indean. Either always execute it, or convert rom to x86 endianess.
    uint16_t Low = mMemory[Address];
    uint16_t High = mMemory[Address+1];

    return uint16_t((High << 8) | Low);
}

void MemoryMapper::Write16Bit(const int32_t Address, uint16_t Value)
{
    uint16_t Low = Value & 0xFF;
    uint16_t High = (Value >> 8) & 0xFF;

    // What happens when it overflows?
    mMemory[Address] = Low;
    mMemory[Address+1] = High;
}

int32_t MemoryMapper::Wrap8Bit(int32_t Address, EAddressingMode AddressingMode)
{
    switch (AddressingMode)
    {
        case EAddressingMode::XIndexedIndirect:
        case EAddressingMode::YIndirectIndexed:
            // Wrap within Zero Page ??
            break;
        default:{
            Address = Address & 0xFF;
        }
    }

    return Address;
}

int32_t MemoryMapper::Wrap16Bit(int32_t Address, EAddressingMode AddressingMode)
{
    switch (AddressingMode)
    {
        case EAddressingMode::XIndexedIndirect:
        case EAddressingMode::YIndirectIndexed:
            // Wrap within Zero Page ??
            break;
        default:{
            Address = Address & 0xFFFF;
        }
    }

    return Address;
}
