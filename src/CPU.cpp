#include "CPU.h"
#include "OpCodeDecoder.h"
#include "RomParameters.h"

#include <array>
#include <cerrno>
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

CPU::CPU()
{
}

void CPU::Init(const ROMData& ROM)
{
    mMasterClockFrequency = MasterClockFrequencies[static_cast<int>(ROM.CPUTimingMode)];
    mClockDivisor = ClockDivisors[static_cast<int>(ROM.CPUTimingMode)];

    mClockFrequency = mMasterClockFrequency / mClockDivisor;
    mCycleTime = 1.0 / mClockFrequency;

    // Supposedly this is always 0xFFFC on power and reboot
    mRegisters.PC = 0xFFFC;

    // INTERRUPT_DISABLE always 1 on power and reboot.
    mRegisters.P |= static_cast<int32_t>(EStatusFlags::INTERRUPT_DISABLE);

    // 0x00 -> 0x03 = 0xFD, whatever that means. Maybe means I need to zero the stack at those offsets?
    // Pointer for 256-byte array whose location is hardcoded at page $01 ($0100-$01FF)
    mRegisters.S = 0xFF;

    // NES uses a decending stack from 0x1FF to 0x100;
    mStackLocation = 0x0100;

    // Need to read ROM to determine memory size and PC location.
    // 6502 can only address up to 64KB due to 16bit address bus.
    // Zero Page: 0x0000 - 0x00FF
    // Stack Page: 0x0100 - 0x01FF
    // Last 6 bytes reserved 0xFFFA - 0xFFFF:
    // non-maskable interrupt handler 0xFFFA - 0xFFFB
    // Power on/Reset 0xFFFC - 0xFFFD
    // BRK/Interrupt Request handler 0xFFFE - 0xFFFF
    mMemoryMapper = std::make_unique<MemoryMapper>(ROM.ChrRomMemory, ROM.PrgRomMemory);

    // PC Code is read from 0xFFFC and 0xFFFD (reset vector)
    mRegisters.PC = mMemoryMapper->Read8Bit(0xFFFC);
    mRegisters.PC |= static_cast<uint16_t>(mMemoryMapper->Read8Bit(0xFFFD)) << 8;

    std::cout << std::format("Initial PC: {0:x}", mRegisters.PC) << std::endl;
}

void CPU::Run()
{
    int64_t TotalCycles = 0;
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

        // Debug Remove me later.
        TotalCycles += CyclesUsed;
        if (TotalCycles > 10)
            break;
    }
}

uint8_t CPU::ExecuteNextInstruction()
{
    const uint8_t PCData = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC++;

    const uint8_t AAA = (PCData >> 5) & (0b111);
    const uint8_t BBB = (PCData >> 2) & (0b111);
    const uint8_t CC = (PCData) & (0b11);

    // Allow OpCode to be modified in the case that cycle count is varies. Ex. Branches, memory reads out of pages, etc.
    auto OpCode = OpCodeDecoder::DecodeOpCode(AAA, BBB, CC);

    ExecuteInstruction(&OpCode);

    // ??
    mRegisters.PC++;

    std::cout << std::format("Executed Instruction: {0}, at {1}, with Addressing Mode {2}", OpCode.Name, mRegisters.PC, static_cast<int32_t>(OpCode.AddressMode)) << std::endl;

    return OpCode.Cycles;
}

void CPU::ExecuteInstruction(NESOpCode* OpCode)
{
    uint8_t OperandsBytes = 0;

    switch (OpCode->Instruction)
    {

        case EINSTRUCTION::BRK:
            BRK();
            break;
        case EINSTRUCTION::CLC:
            CLC();
            break;
        case EINSTRUCTION::ADC:
            ADC(OpCode);
            break;
        case EINSTRUCTION::SBC:
            SBC(OpCode);
            break;
        case EINSTRUCTION::LDA:
            LDA(OpCode);
            break;
        case EINSTRUCTION::LDX:
            LDX(OpCode);
            break;
        case EINSTRUCTION::LDY:
            LDY(OpCode);
            break;
        case EINSTRUCTION::PHA:
            PHA();
            break;
        case EINSTRUCTION::PLA:
            PLA();
            break;
        case EINSTRUCTION::JSR:
            JSR();
            break;
        case EINSTRUCTION::RTS:
            RTS();
            break;
        default:
        {
            std::cout << std::format("Unimplemented OpCode {0}", OpCode->Name) << std::endl;
            std::exit(1);
        }
    }
}

void CPU::PushStack(uint8_t Value)
{
    mMemoryMapper->Write8Bit(mStackLocation + static_cast<uint16_t>(mRegisters.S), Value);
    mRegisters.S--;
}
uint8_t CPU::PopStack()
{
    mRegisters.S++;
    return mMemoryMapper->Read8Bit(mRegisters.S);
}

void CPU::BNE(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::ZERO)) != 0)
    {
        int32_t JumpDistance = Memory;
        if (JumpDistance > 127)
            JumpDistance -= 256;

        mRegisters.PC += JumpDistance;

        OpCode->Cycles += 1;
    }
}

void CPU::BRK()
{
    uint16_t Bytes = mMemoryMapper->Read16Bit(mRegisters.PC);
    mRegisters.PC += 2;

    uint8_t HighByte = (Bytes >> 8) & 0xFF;
    uint8_t LowByte = Bytes & 0xFF;

    PushStack(HighByte);
    PushStack(LowByte);

    // Only set on byte set on stack.
    auto CPUFlags = mRegisters.P;
    CPUFlags |= static_cast<uint8_t>(EStatusFlags::BFlag);

    PushStack(CPUFlags);

    mRegisters.P |= static_cast<uint8_t>(EStatusFlags::INTERRUPT_DISABLE);
    mRegisters.PC = 0xFFFE;
}

void CPU::JSR()
{
    uint16_t Bytes = mMemoryMapper->Read16Bit(mRegisters.PC);
    mRegisters.PC += 1;

    PushStack(static_cast<uint8_t>(mRegisters.PC >> 8));
    PushStack(static_cast<uint8_t>(mRegisters.PC & 0b11111111));
}
void CPU::RTS()
{
    uint8_t LowByte = PopStack();
    uint8_t HighByte = PopStack();

    uint16_t JumpDistance = static_cast<uint16_t>(HighByte) << 8;
    JumpDistance |= LowByte;

    mRegisters.PC += JumpDistance;
    mRegisters.PC++;
}

void CPU::CLC()
{
    mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));
}

void CPU::ADC(const NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    int32_t Carry = (mRegisters.P & static_cast<uint8_t>((EStatusFlags::CARRY))) != 0;

    int32_t A = mRegisters.A + Memory + Carry;

    if (A > 255)
    {
        A = 0;
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    }

    if (A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (const bool bOverflowed = (mRegisters.A ^ A) & (A ^ Memory) & 0x80)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::OVERFLOW));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));

    if (A & 0b01000000)
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.A = A;
}

void CPU::SBC(const NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    int32_t Carry = (mRegisters.P & static_cast<uint8_t>((EStatusFlags::CARRY))) != 0;

    int32_t A = mRegisters.A - Memory - ~Carry;

    if (A < 0)
    {
        A = 255;
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));
    }
    else
    {
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    }

    if (A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (const bool bOverflowed = (mRegisters.A ^ A) & (A ^ ~Memory) & 0x80)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::OVERFLOW));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));

    if (A & 0b01000000)
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.A = A;
}

void CPU::LDA(const NESOpCode* OpCode)
{
    mRegisters.A = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC++;

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b01000000)
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::LDX(const NESOpCode* OpCode)
{
    mRegisters.X = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC++;

    if (mRegisters.X == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.X & 0b01000000)
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::LDY(const NESOpCode* OpCode)
{
    mRegisters.Y = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC++;

    if (mRegisters.Y == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.Y & 0b01000000)
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::PHA()
{
    PushStack(mRegisters.A);
}

void CPU::PLA()
{
    mRegisters.A = PopStack();

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b01000000)
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}
