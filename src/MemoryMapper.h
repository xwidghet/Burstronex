#pragma once

#include <cstdint>
#include <vector>

#include "OpCodeDecoder.h"

class MemoryMapper {
    std::vector<uint8_t> mMemory;

public:
    MemoryMapper(const int32_t MemoryBankSize);

    uint8_t Read8Bit(const int32_t Address);

    void Write8Bit(const int32_t Address, uint8_t Value);

    uint16_t Read16Bit(const int32_t Address);

    void Write16Bit(const int32_t Address, uint16_t Value);

    int32_t Wrap8Bit(int32_t Address, const EAddressingMode AddressingMode);

    int32_t Wrap16Bit(int32_t Address, const EAddressingMode AddressingMode);
};
