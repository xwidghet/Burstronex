#pragma once

#include "MemoryMapper.h"

#include <cstdint>
#include <memory>

struct NESOpCode;
struct ROMData;
enum class ECPU_TIMING;

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

    uint16_t mStackLocation = 0;

    double mMasterClockFrequency = 0;
    double mClockDivisor = 0;
    double mClockFrequency = 0.f;
    double mCycleTime = 0.f;

    std::unique_ptr<MemoryMapper> mMemoryMapper;

public :
    CPU();

    // Initialize the CPU to begin running at the given Program Code location.
    void Init(const ROMData& ROM);

    void Run();

    // Executes the instruction at the current PC and increments the PC based on the instruction's data and execution etc.
    // Returns cycles used.
    uint8_t ExecuteNextInstruction();

    void ExecuteInstruction(NESOpCode* OpCode);

// Should probably move instructions to a different file, but this'll do for now.
private:
    void PushStack(uint8_t Value);
    uint8_t PopStack();

    void BNE(NESOpCode* OpCode);
    void BRK();
    void JSR();
    void RTS();
    void CLC();
    void ADC(const NESOpCode* OpCode);
    void SBC(const NESOpCode* OpCode);
    void LDA(const NESOpCode* OpCode);
    void LDX(const NESOpCode* OpCode);
    void LDY(const NESOpCode* OpCode);
    void PHA();
    void PLA();
};
