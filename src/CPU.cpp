#include "CPU.h"
#include "OpCodeDecoder.h"
#include "RomParameters.h"

#include <array>
#include <chrono>
#include <format>
#include <iostream>

std::array<double, 4> MasterClockFrequencies =
{
    // NTSC
    21477272,
    // PAL
    26601712,
    // DENDY
    26601712,
    // PC
    21441960
};

std::array<double, 4> ClockDivisors =
{
    // NTSC
    12,
    // PAL
    16,
    // DENDY
    15,
    // PC
    12
};

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

CPU::CPU(ECPU_TIMING Timing)
{
    mMasterClockFrequency = MasterClockFrequencies[static_cast<int>(Timing)];
    mClockDivisor = ClockDivisors[static_cast<int>(Timing)];

    mClockFrequency = mMasterClockFrequency / mClockDivisor;
    mCycleTime = 1.0 / mClockFrequency;
}

void CPU::Init(const uint16_t ProgramCodeLocation, const ROMData& ROM)
{
    mRegisters.PC = ProgramCodeLocation;

    // What to do with trainer memory, do we need to increase size to load it?
    int32_t TotalROMSize = ROM.ChrRomMemory.size() + ROM.PrgRomMemory.size();

    // Need to read ROM to determine memory size and PC location.
    // 6502 can only address up to 64KB due to 16bit address bus.
    // Zero Page: 0x0000 - 0x00FF
    // Stack Page: 0x0100 - 0x01FF
    // Last 6 bytes reserved 0xFFFA - 0xFFFF:
    // non-maskable interrupt handler 0xFFFA - 0xFFFB
    // Power on/Reset 0xFFFC - 0xFFFD
    // BRK/Interrupt Request handler 0xFFFE - 0xFFFF
    mMemoryMapper = std::make_unique<MemoryMapper>(TotalROMSize);
}

void CPU::Run()
{
    while(true)
    {
        auto TimeAtInstruction = std::chrono::steady_clock::now();

        auto CyclesUsed = ExecuteNextInstruction();

        auto ExecutionTime = mCycleTime * CyclesUsed;
        auto TimePassed = std::chrono::steady_clock::now() - TimeAtInstruction;

        double DeltaTime = std::chrono::duration<double>(TimePassed).count();
        while(DeltaTime < ExecutionTime)
        {
            TimePassed = std::chrono::steady_clock::now() - TimeAtInstruction;
            DeltaTime = std::chrono::duration<double>(TimePassed).count();
        }
    }
}

uint8_t CPU::ExecuteNextInstruction()
{
    const uint8_t PCData = mMemoryMapper->Read8Bit(mRegisters.PC);

    const uint8_t AAA = (PCData >> 5) & (0b111);
    const uint8_t BBB = (PCData >> 2) & (0b111);
    const uint8_t CC = (PCData) & (0b11);

    uint8_t ExecutionBytes = 1;

    const auto OpCode = OpCodeDecoder::DecodeOpCode(AAA, BBB, CC);
    ExecutionBytes += OpCode.OperandByteCount;

    ExecuteInstruction(&OpCode);

    // How do we handle Jumps???
    // Since there's only one jump instruction...could just not increment if we've jumped *shrug*.
    mRegisters.PC += ExecutionBytes;

    return OpCode.Cycles;
}

void  CPU::ExecuteInstruction(const NESOpCode* OpCode)
{
    uint8_t OperandsBytes = 0;

    switch (OpCode->Instruction)
    {
        case EINSTRUCTION::BRK:
            break;
        case EINSTRUCTION::PHP:
            break;
        case EINSTRUCTION::BPL:
            break;
        case EINSTRUCTION::CLC:
            break;
        case EINSTRUCTION::JSR:
            break;
        case EINSTRUCTION::BIT:
            break;
        case EINSTRUCTION::PLP:
            break;
        case EINSTRUCTION::BMI:
            break;
        case EINSTRUCTION::SEC:
            break;
        case EINSTRUCTION::RTI:
            break;
        case EINSTRUCTION::PHA:
            break;
        case EINSTRUCTION::JMP:
            break;
        case EINSTRUCTION::BVC:
            break;
        case EINSTRUCTION::CLI:
            break;
        case EINSTRUCTION::RTS:
            break;
        case EINSTRUCTION::PLA:
            break;
        case EINSTRUCTION::BVS:
            break;
        case EINSTRUCTION::SEI:
            break;
        case EINSTRUCTION::STY:
            break;
        case EINSTRUCTION::DEY:
            break;
        case EINSTRUCTION::BCC:
            break;
        case EINSTRUCTION::TYA:
            break;
        case EINSTRUCTION::SHY:
            break;
        case EINSTRUCTION::LDY:
            break;
        case EINSTRUCTION::TAY:
            break;
        case EINSTRUCTION::BCS:
            break;
        case EINSTRUCTION::CLV:
            break;
        case EINSTRUCTION::CPY:
            break;
        case EINSTRUCTION::INY:
            break;
        case EINSTRUCTION::BNE:
            break;
        case EINSTRUCTION::CLD:
            break;
        case EINSTRUCTION::CPX:
            break;
        case EINSTRUCTION::INX:
            break;
        case EINSTRUCTION::BEQ:
            break;
        case EINSTRUCTION::SED:
            break;
        case EINSTRUCTION::NOP:
            break;
        case EINSTRUCTION::ORA:
            break;
        case EINSTRUCTION::AND:
            break;
        case EINSTRUCTION::EOR:
            break;
        case EINSTRUCTION::ADC:
            break;
        case EINSTRUCTION::STA:
            break;
        case EINSTRUCTION::LDA:
            break;
        case EINSTRUCTION::CMP:
            break;
        case EINSTRUCTION::SBC:
            break;
        case EINSTRUCTION::STP:
            break;
        case EINSTRUCTION::ASL:
            break;
        case EINSTRUCTION::ROL:
            break;
        case EINSTRUCTION::LSR:
            break;
        case EINSTRUCTION::ROR:
            break;
        case EINSTRUCTION::STX:
            break;
        case EINSTRUCTION::TXA:
            break;
        case EINSTRUCTION::TXS:
            break;
        case EINSTRUCTION::SHX:
            break;
        case EINSTRUCTION::LDX:
            break;
        case EINSTRUCTION::TAX:
            break;
        case EINSTRUCTION::TSX:
            break;
        case EINSTRUCTION::DEC:
            break;
        case EINSTRUCTION::DEX:
            break;
        case EINSTRUCTION::INC:
            break;
        case EINSTRUCTION::SLO:
            break;
        case EINSTRUCTION::ANC:
            break;
        case EINSTRUCTION::RLA:
            break;
        case EINSTRUCTION::SRE:
            break;
        case EINSTRUCTION::ALR:
            break;
        case EINSTRUCTION::RRA:
            break;
        case EINSTRUCTION::ARR:
            break;
        case EINSTRUCTION::SAX:
            break;
        case EINSTRUCTION::XAA:
            break;
        case EINSTRUCTION::AHX:
            break;
        case EINSTRUCTION::TAS:
            break;
        default:
        {
            std::cout << std::format("Unimplemented OpCode {0}", OpCode->Name) << std::endl;
        }
    }
}
