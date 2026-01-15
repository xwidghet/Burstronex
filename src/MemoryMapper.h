#pragma once

#include <cstdint>
#include <vector>

#include "OpCodeDecoder.h"

class CPU;
class PPU;

class MemoryMapper {
    std::vector<uint8_t> mMemory;

    uint16_t mPrgRomLocation;
    uint16_t mChrRomLocation;

    std::vector<char> mChrRomMemory;
    std::vector<char> mPrgRomMemory;

    CPU* mCPU;
    PPU* mPPU;

public:
    MemoryMapper(const std::vector<char>& ChrRomMemory, const std::vector<char>& PrgRomMemory);

    void Init(CPU* CPU, PPU* PPU);

    uint32_t MapAddress(uint32_t Address);

    uint8_t Read8Bit(const uint32_t Address);

    void Write8Bit(const uint32_t Address, uint8_t Value);

    uint16_t Read16Bit(const uint32_t Address);

    void Write16Bit(const uint32_t Address, uint16_t Value);

    // Used for APU/PPU to avoid read/write side-effects, since the memory is supposed to be mapped to them.
    uint8_t ReadRegister(const uint32_t Address);

    // Used for APU/PPU to avoid read/write side-effects, since the memory is supposed to be mapped to them.
    void WriteRegister(const uint32_t Address, uint8_t Value);

    const uint16_t GetPrgRomLocation() const;

    const uint16_t GetChrRomLocation() const;
};
