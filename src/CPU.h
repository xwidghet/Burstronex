#pragma once

#include "MemoryMapper.h"

#include <cstdint>
#include <memory>

struct NESOpCode;

enum class CPU_TIMING
{
    NTSC,
    PAL,
    DENDY,
    PC
};

class CPU {
    // Are NES CPU Registers zero initialized??
    struct Registers {
        // Accumulator - Byte-wide. Supports status register for carrying, overflow detection, etc.
        uint8_t A = 0;

        // Indexes Byte-wide, used for loop counters, INC/DEC and branch instructions.
        uint8_t X = 0;

        // Indexes Byte-wide, used for loop counters, INC/DEC and branch instructions.
        uint8_t Y = 0;

        // Program Counter 2-Byte-wide (65536) unbanked memory locations. Accessed via CPU increment address bus, interrupt (NMI, Reset, IRQ/BRQ), and RTS/JMP/JSR/Branch instructions
        uint16_t PC = 0;

        // Stack Pointer Byte-wide, accessed using interrupts, pulls, pushes, and transfers. Indexes int 256-byte stack at $0100-$01FF.
        uint8_t S = 0;

        // Status Register Byte-wide but 6 bits used by ALU. PHP, PLP, arithmetic, testing and branch instructions can access this register.
        uint8_t P = 0;
    } mRegisters;

    double mMasterClockFrequency = 0;
    double mClockDivisor = 0;
    double mClockFrequency = 0.f;
    double mCycleTime = 0.f;

    std::unique_ptr<MemoryMapper> mMemoryMapper;

public :
    CPU(CPU_TIMING Timing);

    // Initialize the CPU to begin running at the given Program Code location.
    void Init(const uint16_t ProgramCodeLocation);

    void Run();

    // Executes the instruction at the current PC and increments the PC based on the instruction's data and execution etc.
    // Returns cycles used.
    uint8_t ExecuteNextInstruction();

    // Returns number of bytes used by operands to allow for incrementing PC.
    void ExecuteInstruction(const NESOpCode* OpCode);
};
