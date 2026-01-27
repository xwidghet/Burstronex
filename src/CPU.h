#pragma once

#include "OpCodeDecoder.h"

#include <cstdint>
#include <memory>

struct NESOpCode;
struct ROMData;
enum class ECPU_TIMING;
class MemoryMapper;
class PPU;

// 0xFFFA - 0xFFFB
static const uint16_t NMI_HANDLER_ADDRESS = 0xFFFA;

// 0xFFFE–0xFFFF
static const uint16_t IRQ_HANDLER_ADDRESS = 0xFFFE;

static const uint16_t INITIAL_PC = 0xFFFC;

// Windows compile fix. TODO: Remove once no iostream debug usage is in this file.
#undef OVERFLOW

enum class EStatusFlags : uint8_t {
    CARRY = 1 << 0,
    ZERO = 1 << 1,
    INTERRUPT_DISABLE = 1 << 2,
    DECIMAL = 1 << 3,
    // No CPU effect ??
    BFlag = 1 << 4,
    // No CPU effect ??
    ONEFLAG = 1 << 5,
    OVERFLOW = 1 << 6,
    NEGATIVE =  1 << 7
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

    uint16_t mStackLocation = 0;

    double mMasterClockFrequency = 0;
    double mClockDivisor = 0;
    double mClockFrequency = 0.f;
    double mCycleTime = 0.f;

    int64_t mCycleCount = 0;

    bool mbFrameCounterIRQTriggered = false;
    bool mbDMCIRQTriggered = false;

    MemoryMapper* mMemoryMapper;

    // Needed for reading the NMI Pin, could refactor this later with like...a pin manager?
    PPU* mPPU;

public :
    CPU();

    // Initialize the CPU to begin running at the given Program Code location.
    void Init(const ROMData& ROM, MemoryMapper* MemoryMapper, PPU* PPU);

    // Executes the instruction at the current PC and increments the PC based on the instruction's data and execution etc.
    // Returns cycles used.
    uint8_t ExecuteNextInstruction();

    double GetCycleTime() const;

    double GetClockFrequency() const;

    int64_t GetCycleCount() const;

    void SetFrameCounterIRQ(bool bValue);

    void SetDMCIRQ(bool bValue);

// Should probably move instructions to a different file, but this'll do for now.
private:
    void ExecuteInstruction(NESOpCode* OpCode);

    void TriggerInterrupt();

    // Automate setting data test roms expect for input-less running.
    void DebugInit(const ROMData& ROM);

    void PushStack(uint8_t Value);
    uint8_t PopStack();

    // Using the current PC, reads the current operand's addressing parameters and writes the given value to the address calculated.
    // Increments PC as needed based on Address Mode.
    void WriteMemory(NESOpCode* OpCode, uint8_t Value);

    // Using the current PC, reads the current operand's addressing parameters and returns the value from the address calculated.
    // Increments PC as needed based on Address Mode.
    // Increments OpCode's cycle count when a page is crossed.
    uint8_t ReadMemory(NESOpCode* OpCode);

    static bool PageCrossed(const uint16_t Left, const uint16_t Right);

    void SetStatusBit(const EStatusFlags StatusFlag, const bool Enabled);

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
    void JMP(NESOpCode* OpCode);
    void JSR();
    void RTS();
    void RTI();
    void CMP(NESOpCode* OpCode);
    void CPX(NESOpCode* OpCode);
    void CPY(NESOpCode* OpCode);
    void CLC();
    void CLD();
    void CLV();
    void SEC();
    void SED();
    void SEI();
    void CLI();
    void AND(NESOpCode* OpCode);
    void ORA(NESOpCode* OpCode);
    void ADC(NESOpCode* OpCode);
    void SBC(NESOpCode* OpCode);
    void DEC(NESOpCode* OpCode);
    void DEX();
    void DEY();
    void EOR(NESOpCode* OpCode);
    void INC(NESOpCode* OpCode);
    void INX();
    void INY();
    void LDA(NESOpCode* OpCode);
    void LDX(NESOpCode* OpCode);
    void LDY(NESOpCode* OpCode);
    void PHA();
    void PHP();
    void PLA();
    void PLP();
    void STA(NESOpCode* OpCode);
    void STX(NESOpCode* OpCode);
    void STY(NESOpCode* OpCode);
    void TAX();
    void TAY();
    void TSX();
    void TXS();
    void TXA();
    void TYA();
    void NOP(NESOpCode* OpCode);
    void ALR(NESOpCode* OpCode);
    void ANC(NESOpCode* OpCode);
    void ARR(NESOpCode* OpCode);
    void AXS(NESOpCode* OpCode);
    void LAX(NESOpCode* OpCode);
    void SAX(NESOpCode* OpCode);
    void DCP(NESOpCode* OpCode);
    void ISC(NESOpCode* OpCode);
    void RLA(NESOpCode* OpCode);
    void RRA(NESOpCode* OpCode);
    void SLO(NESOpCode* OpCode);
    void SRE(NESOpCode* OpCode);
};
