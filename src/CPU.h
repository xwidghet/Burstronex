#pragma once

#include "MemoryMapper.h"
#include "OpCodeDecoder.h"

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
    // Automate setting data test roms expect for input-less running.
    void DebugInit(const ROMData& ROM);

    void PushStack(uint8_t Value);
    uint8_t PopStack();

    // Using the current PC, reads the current operand's addressing parameters and writes the given value to the address calculated.
    // Increments PC as needed based on Address Mode.
    void WriteMemory(EAddressingMode AddressMode, uint8_t Value);

    // Using the current PC, reads the current operand's addressing parameters and returns the value from the address calculated.
    // Increments PC as needed based on Address Mode.
    uint8_t ReadMemory(EAddressingMode AddressMode);

    void ASL(NESOpCode* OpCode);
    void LSR(NESOpCode* OpCode);
    void ROL(NESOpCode* OpCode);
    void ROR(NESOpCode* OpCode);
    void BNE(NESOpCode* OpCode);
    void BMI(NESOpCode* OpCode);
    void BPL(NESOpCode* OpCode);
    void BCC(NESOpCode* OpCode);
    void BCS(NESOpCode* OpCode);
    void BEQ(NESOpCode* OpCode);
    void BVC(NESOpCode* OpCode);
    void BVS(NESOpCode* OpCode);
    void BIT(NESOpCode* OpCode);
    void BRK();
    void JMP(const NESOpCode* OpCode);
    void JSR();
    void RTS();
    void CMP(NESOpCode* OpCode);
    void CPX(const NESOpCode* OpCode);
    void CPY(const NESOpCode* OpCode);
    void CLC();
    void CLD();
    void CLV();
    void SEC();
    void SED();
    void SEI();
    void AND(const NESOpCode* OpCode);
    void ADC(const NESOpCode* OpCode);
    void SBC(const NESOpCode* OpCode);
    void DEC(const NESOpCode* OpCode);
    void DEX();
    void DEY();
    void EOR(const NESOpCode* OpCode);
    void INC(const NESOpCode* OpCode);
    void INX();
    void INY();
    void LDA(const NESOpCode* OpCode);
    void LDX(const NESOpCode* OpCode);
    void LDY(const NESOpCode* OpCode);
    void PHA();
    void PHP();
    void PLA();
    void PLP();
    void STA(const NESOpCode* OpCode);
    void STX(const NESOpCode* OpCode);
    void STY(const NESOpCode* OpCode);
    void TAX();
    void TAY();
    void TSX();
    void TXS();
    void TXA();
    void TYA();
};
