#pragma once

#include <cstdint>
#include <vector>

#include "OpCodeDecoder.h"

class MemoryMapper {
    std::vector<uint8_t> mMemory;

    uint16_t mPrgRomLocation;
    uint16_t mChrRomLocation;

    std::vector<char> mChrRomMemory;
    std::vector<char> mPrgRomMemory;

public:
    MemoryMapper(const std::vector<char>& ChrRomMemory, const std::vector<char>& PrgRomMemory);

    uint32_t MapAddress(uint32_t Address);

    uint8_t Read8Bit(const uint32_t Address);

    void Write8Bit(const uint32_t Address, uint8_t Value);

    uint16_t Read16Bit(const uint32_t Address);

    void Write16Bit(const uint32_t Address, uint16_t Value);

    uint32_t Wrap8Bit(uint32_t Address, const EAddressingMode AddressingMode);

    uint32_t Wrap16Bit(uint32_t Address, const EAddressingMode AddressingMode);

    const uint16_t GetPrgRomLocation() const;

    const uint16_t GetChrRomLocation() const;
};
