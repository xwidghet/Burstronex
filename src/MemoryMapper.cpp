#include "MemoryMapper.h"
#include "OpCodeDecoder.h"

MemoryMapper::MemoryMapper(const int32_t MemoryBankSize)
{
    mMemory = std::vector<uint8_t>(MemoryBankSize);
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
    uint16_t Data = mMemory[Address];
    uint16_t Low = Data & 0xFF;
    uint16_t High = (Data >> 8) & 0xFF;

    return uint16_t((High << 8) & Low);
}

void MemoryMapper::Write16Bit(const int32_t Address, uint16_t Value)
{
    uint16_t Low = Value & 0xFF;
    uint16_t High = (Value >> 8) & 0xFF;

    mMemory[Address] = uint16_t((High << 8) & Low);
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
