#include "CPU.h"

#include "MemoryMapper.h"
#include "OpCodeDecoder.h"
#include "RomParameters.h"
#include "PPU.h"

#include <array>
#include <cassert>
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

void CPU::Init(const ROMData& ROM, MemoryMapper* MemoryMapper, PPU* PPU)
{
    mMasterClockFrequency = MasterClockFrequencies[static_cast<int>(ROM.CPUTimingMode)];
    mClockDivisor = ClockDivisors[static_cast<int>(ROM.CPUTimingMode)];

    mClockFrequency = mMasterClockFrequency / mClockDivisor;
    mCycleTime = 1.0 / mClockFrequency;

    // Supposedly this is always 0xFFFC on power and reboot
    mRegisters.PC = INITIAL_PC;

    // INTERRUPT_DISABLE always 1 on power and reboot.
    mRegisters.P = 0x24;

    // 0x00 -> 0x03 = 0xFD, whatever that means. Maybe means I need to zero the stack at those offsets?
    // Pointer for 256-byte array whose location is hardcoded at page $01 ($0100-$01FF)
    mRegisters.S = 0xFD;

    // NES uses a decending stack from 0x1FF to 0x100;
    mStackLocation = 0x0100;

    // 6502 can only address up to 64KB due to 16bit address bus.
    // Zero Page: 0x0000 - 0x00FF
    // Stack Page: 0x0100 - 0x01FF
    // Last 6 bytes reserved 0xFFFA - 0xFFFF:
    // non-maskable interrupt handler 0xFFFA - 0xFFFB
    // Power on/Reset 0xFFFC - 0xFFFD
    // BRK/Interrupt Request handler 0xFFFE - 0xFFFF
    mMemoryMapper = MemoryMapper;

    mPPU = PPU;

    // PC Code is read from 0xFFFC and 0xFFFD (reset vector)
    mRegisters.PC = mMemoryMapper->Read16Bit(INITIAL_PC);

    //std::cout << std::format("FFFC: {0:x}", mMemoryMapper->Read8Bit(0xFFFB)) << std::endl;
    //std::cout << std::format("FFFD: {0:x}", mMemoryMapper->Read8Bit(0xFFFC)) << std::endl;

    DebugInit(ROM);

    std::cout << std::format("Initial PC: {0:x}", mRegisters.PC) << std::endl;
}

void CPU::DebugInit(const ROMData& ROM)
{
    if (ROM.Name.compare("nestest") == 0)
    {
        // nestest automation PC. Outputs final results in 0x02 and 0x03
        mRegisters.PC = 0xC000;
    }
}

double CPU::GetCycleTime() const
{
    return mCycleTime;
}

uint8_t CPU::ExecuteNextInstruction()
{
    // Interrupts
    // This flag's behavior should be delayed by one frame somehow
    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::INTERRUPT_DISABLE)) == 0)
    {
        // Other Interrupts (IRQs)
    }

    // Non-Maskable Interrupts cannot be ignored by the CPU
    if (mPPU->ReadNMIOutput())
    {
        TriggerInterrupt();

        uint8_t PPUCTRL = mMemoryMapper->Read8Bit(PPUCTRL_ADDRESS);
        PPUCTRL &= ~static_cast<uint8_t>(EPPUCTRL::VBLANK_NMI_ENABLE);
        mMemoryMapper->Write8Bit(PPUCTRL_ADDRESS, PPUCTRL);

        // NMI Handler
        // Finishes last instruction, then jumps to this address immediately
        // Since the PPU will tick three times after the CPU finishes the last instruction
        // this should be triggered.
        mRegisters.PC = mMemoryMapper->Read16Bit(NMI_HANDLER_ADDRESS);
    }

    // Debug
    auto InstructionPC = mRegisters.PC;

    const uint8_t PCData = mMemoryMapper->Read8Bit(mRegisters.PC);

    mRegisters.PC++;

    const uint8_t AAA = (PCData >> 5) & (0b111);
    const uint8_t BBB = (PCData >> 2) & (0b111);
    const uint8_t CC = (PCData) & (0b11);

    // Allow OpCode to be modified in the case that cycle count is varies. Ex. Branches, memory reads out of pages, etc.
    auto OpCode = OpCodeDecoder::DecodeOpCode(AAA, BBB, CC);

    std::cout << std::format("Executing Instruction: {0} ({3:X}), at {1:X}, with Addressing Mode {2}", OpCode.Name, InstructionPC, static_cast<int32_t>(OpCode.AddressMode), PCData) << std::endl;

    ExecuteInstruction(&OpCode);
    std::cout << std::format("Registers: A:{0:X}, X:{1:X}, Y:{2:X}, PC:{3:X}, S:{4:X}, P:{5:X}",
                             mRegisters.A, mRegisters.X, mRegisters.Y, mRegisters.PC, mRegisters.S, mRegisters.P) << std::endl;

    return OpCode.Cycles;
}

void CPU::ExecuteInstruction(NESOpCode* OpCode)
{
    uint8_t OperandsBytes = 0;

    switch (OpCode->Instruction)
    {
        case EINSTRUCTION::ASL:
            ASL(OpCode);
            break;
        case EINSTRUCTION::LSR:
            LSR(OpCode);
            break;
        case EINSTRUCTION::ROL:
            ROL(OpCode);
            break;
        case EINSTRUCTION::ROR:
            ROR(OpCode);
            break;
        case EINSTRUCTION::BRK:
            BRK();
            break;
        case EINSTRUCTION::BMI:
            BMI(OpCode);
            break;
        case EINSTRUCTION::BNE:
            BNE(OpCode);
            break;
        case EINSTRUCTION::BPL:
            BPL(OpCode);
            break;
        case EINSTRUCTION::BCC:
            BCC(OpCode);
            break;
        case EINSTRUCTION::BCS:
            BCS(OpCode);
            break;
        case EINSTRUCTION::BEQ:
            BEQ(OpCode);
            break;
        case EINSTRUCTION::BVC:
            BVC(OpCode);
            break;
        case EINSTRUCTION::BVS:
            BVS(OpCode);
            break;
        case EINSTRUCTION::BIT:
            BIT(OpCode);
            break;
        case EINSTRUCTION::JMP:
            JMP(OpCode);
            break;
        case EINSTRUCTION::JSR:
            JSR();
            break;
        case EINSTRUCTION::RTS:
            RTS();
            break;
        case EINSTRUCTION::RTI:
            RTI();
            break;
        case EINSTRUCTION::CMP:
            CMP(OpCode);
            break;
        case EINSTRUCTION::CPX:
            CPX(OpCode);
            break;
        case EINSTRUCTION::CPY:
            CPY(OpCode);
            break;
        case EINSTRUCTION::SEI:
            SEI();
            break;
        case EINSTRUCTION::SED:
            SED();
            break;
        case EINSTRUCTION::CLD:
            CLD();
            break;
        case EINSTRUCTION::CLC:
            CLC();
            break;
        case EINSTRUCTION::CLV:
            CLV();
            break;
        case EINSTRUCTION::AND:
            AND(OpCode);
            break;
        case EINSTRUCTION::ORA:
            ORA(OpCode);
            break;
        case EINSTRUCTION::ADC:
            ADC(OpCode);
            break;
        case EINSTRUCTION::SEC:
            SEC();
            break;
        case EINSTRUCTION::SBC:
            SBC(OpCode);
            break;
        case EINSTRUCTION::DEC:
            DEC(OpCode);
            break;
        case EINSTRUCTION::DEX:
            DEX();
            break;
        case EINSTRUCTION::DEY:
            DEY();
            break;
        case EINSTRUCTION::EOR:
            EOR(OpCode);
            break;
        case EINSTRUCTION::INC:
            INC(OpCode);
            break;
        case EINSTRUCTION::INX:
            INX();
            break;
        case EINSTRUCTION::INY:
            INY();
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
        case EINSTRUCTION::PHP:
            PHP();
            break;
        case EINSTRUCTION::PLA:
            PLA();
            break;
        case EINSTRUCTION::PLP:
            PLP();
            break;
        case EINSTRUCTION::STA:
            STA(OpCode);
            break;
        case EINSTRUCTION::STX:
            STX(OpCode);
            break;
        case EINSTRUCTION::STY:
            STY(OpCode);
            break;
        case EINSTRUCTION::TAX:
            TAX();
            break;
        case EINSTRUCTION::TAY:
            TAY();
            break;
        case EINSTRUCTION::TSX:
            TSX();
            break;
        case EINSTRUCTION::TXS:
            TXS();
            break;
        case EINSTRUCTION::TXA:
            TXA();
            break;
        case EINSTRUCTION::TYA:
            TYA();
            break;
        case EINSTRUCTION::NOP:
            NOP();
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
    return mMemoryMapper->Read8Bit(mStackLocation + static_cast<uint16_t>(mRegisters.S));
}

void CPU::WriteMemory(EAddressingMode AddressMode, uint8_t Value)
{
    // Not sure how to implement these, so asserting to find use cases.
    assert(AddressMode != EAddressingMode::Implicit);
    assert(AddressMode != EAddressingMode::Immediate);

    uint8_t OperandLowByte = 0;
    uint8_t OperandHighByte = 0;

    // Only used for...relative Addressing Mode.
    int32_t RelativeOperand = 0;

    uint16_t Address = 0;

    switch (AddressMode)
    {
        // d,x
        case EAddressingMode::XZeroPageIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            // Wraps around 0x00 -> 0xFF
            Address = (int32_t(OperandLowByte) + mRegisters.X) & 0xFF;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // d,y
        case EAddressingMode::YZeroPageIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            // Wraps around 0x00 -> 0xFF
            Address = (int32_t(OperandLowByte) + mRegisters.Y) & 0xFF;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // a,x
        case EAddressingMode::XAbsoluteIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;
            OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;
            Address += mRegisters.X;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // a,y
        case EAddressingMode::YAbsoluteIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;
            OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;
            Address += mRegisters.Y;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // (d, x)
        case EAddressingMode::XIndexedIndirect:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (OperandLowByte + mRegisters.X) & 0xFF;
            OperandLowByte = mMemoryMapper->Read8Bit(Address);

            Address = (OperandLowByte + mRegisters.X + 1) & 0xFF;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) + OperandLowByte;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // (d), y
        case EAddressingMode::YIndirectIndexed:
            Address = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            OperandLowByte = mMemoryMapper->Read8Bit(Address);

            Address = (Address + 1) & 256;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) + OperandLowByte;
            Address += mRegisters.Y;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // No address operand, ex. RTS / CLC
        case EAddressingMode::Implicit:
            break;
        // A
        case EAddressingMode::Accumulator:
            mRegisters.A = Value;
            break;
        // #i (Wiki called it #v for some unknown reason????)
        case EAddressingMode::Immediate:
            // What is immediate mode for writting?
            break;
        // d
        case EAddressingMode::Zeropage:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            mMemoryMapper->Write8Bit(0x00 + OperandLowByte, Value);
            break;
        // a
        case EAddressingMode::Absolute:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;
            OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // *+d (label??)
        // Only used by branch instructions, so this *should* be unused.
        case EAddressingMode::Relative:
            RelativeOperand = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            if (RelativeOperand > 127)
                RelativeOperand -= 256;

            Address = mRegisters.PC + RelativeOperand;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // (a)
        case EAddressingMode::Indirect:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;
            OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            // 16 bit operand which points to another 16bit address which is the real target.
            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

            OperandLowByte = mMemoryMapper->Read8Bit(Address);
            Address++;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
    }
}

uint8_t CPU::ReadMemory(EAddressingMode AddressMode)
{
    assert(AddressMode != EAddressingMode::Implicit);

    uint8_t OperandLowByte = 0;
    uint8_t OperandHighByte = 0;

    // Only used for...relative Addressing Mode.
    int32_t RelativeOperand = 0;

    uint16_t Address = 0;

    switch (AddressMode)
    {
        // d,x
        case EAddressingMode::XZeroPageIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            // Wraps around 0x00 -> 0xFF
            Address = (int32_t(OperandLowByte) + mRegisters.X) & 0xFF;

            return mMemoryMapper->Read8Bit(Address);
            break;
            // d,y
        case EAddressingMode::YZeroPageIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            // Wraps around 0x00 -> 0xFF
            Address = (int32_t(OperandLowByte) + mRegisters.Y) & 0xFF;

            return mMemoryMapper->Read8Bit(Address);
            break;
            // a,x
        case EAddressingMode::XAbsoluteIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;
            OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;
            Address += mRegisters.X;

            return mMemoryMapper->Read8Bit(Address);
            break;
            // a,y
        case EAddressingMode::YAbsoluteIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;
            OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;
            Address += mRegisters.Y;

            return mMemoryMapper->Read8Bit(Address);
            break;
            // (d, x)
        case EAddressingMode::XIndexedIndirect:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (OperandLowByte + mRegisters.X) & 0xFF;
            OperandLowByte = mMemoryMapper->Read8Bit(Address);

            Address = (OperandLowByte + mRegisters.X + 1) & 0xFF;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) + OperandLowByte;

            return mMemoryMapper->Read8Bit(Address);
            break;
            // (d), y
        case EAddressingMode::YIndirectIndexed:
            Address = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            OperandLowByte = mMemoryMapper->Read8Bit(Address);

            Address = (Address + 1) & 256;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) + OperandLowByte;
            Address += mRegisters.Y;

            return mMemoryMapper->Read8Bit(Address);
            break;
            // No address operand, ex. RTS / CLC
        case EAddressingMode::Implicit:
            // Should never be called.
            break;
            // A
        case EAddressingMode::Accumulator:
            return mRegisters.A;
            break;
            // #i (Wiki called it #v for some unknown reason????)
        case EAddressingMode::Immediate:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            return OperandLowByte;
            break;
            // d
        case EAddressingMode::Zeropage:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = 0x00 + OperandLowByte;

            return mMemoryMapper->Read8Bit(Address);
            break;
            // a
        case EAddressingMode::Absolute:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;
            OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | static_cast<uint16_t>(OperandLowByte);
            std::cout << std::format("Absolute state of Address: {0:X}", Address) << std::endl;
            std::cout << std::format("Value: {0:X}", mMemoryMapper->Read8Bit(Address)) << std::endl;
            return mMemoryMapper->Read8Bit(Address);
            break;
            // *+d (label??), only used by jump commands directly
        case EAddressingMode::Relative:
            RelativeOperand = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            if (RelativeOperand > 127)
                RelativeOperand -= 256;

            Address = mRegisters.PC + RelativeOperand;

            return mMemoryMapper->Read8Bit(Address);
            break;
            // (a)
        case EAddressingMode::Indirect:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;
            OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            // 16 bit operand which points to another 16bit address which is the real target.
            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

            OperandLowByte = mMemoryMapper->Read8Bit(Address);
            Address++;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

            return mMemoryMapper->Read8Bit(Address);
            break;
    }

    // Should Never Happen TM
    return 0;
}

void CPU::TriggerInterrupt()
{
    uint8_t HighByte = (mRegisters.PC >> 8) & 0xFF;
    uint8_t LowByte = mRegisters.PC & 0xFF;

    PushStack(HighByte);
    PushStack(LowByte);

    // Both IRQ and NMI set this to zero before pushing, while BRK and PHP set 1.
    auto CPUFLAGS = mRegisters.P;
    CPUFLAGS &= ~static_cast<uint8_t>(EStatusFlags::BFlag);
    PushStack(CPUFLAGS);

    // Only set outside of stack
    mRegisters.P |= static_cast<uint8_t>(EStatusFlags::INTERRUPT_DISABLE);
}

void CPU::ASL(NESOpCode* OpCode)
{
    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode->AddressMode);
    bool Carry = (Memory & 0b10000000) != 0;

    Memory = Memory << 1;

    if (Carry)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    if (Memory == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (Memory & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode->AddressMode, Memory);
}

void CPU::LSR(NESOpCode* OpCode)
{
    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode->AddressMode);
    bool Carry = (Memory & 0b00000001) != 0;

    Memory = Memory >> 1;

    if (Carry)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    if (Memory == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    // Since Bit 7 is always zero after RHS, this is always set to zero.
    mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode->AddressMode, Memory);
}

void CPU::ROL(NESOpCode* OpCode)
{
    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode->AddressMode);
    bool MemoryCarry = (Memory & 0b10000000) != 0;
    bool CPUCarry = (mRegisters.P & static_cast<uint8_t>(EStatusFlags::CARRY)) != 0;

    Memory = Memory << 1;

    Memory |= static_cast<uint8_t>(CPUCarry);

    if (MemoryCarry)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    if (Memory == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (Memory & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode->AddressMode, Memory);
}

void CPU::ROR(NESOpCode* OpCode)
{
    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode->AddressMode);
    bool MemoryCarry = (Memory & 0b00000001) != 0;
    bool CPUCarry = (mRegisters.P & static_cast<uint8_t>(EStatusFlags::CARRY)) != 0;

    Memory = Memory >> 1;

    Memory |= (static_cast<uint8_t>(CPUCarry) << 7);

    if (MemoryCarry)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    if (Memory == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (Memory & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode->AddressMode, Memory);
}

void CPU::BNE(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::ZERO)) == 0)
    {
        int32_t JumpDistance = Memory;
        if (JumpDistance > 127)
            JumpDistance -= 256;

        mRegisters.PC += JumpDistance;

        OpCode->Cycles += 1;
    }
}

void CPU::BPL(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::NEGATIVE)) == 0)
    {
        int32_t JumpDistance = Memory;
        if (JumpDistance > 127)
            JumpDistance -= 256;

        mRegisters.PC += JumpDistance;

        OpCode->Cycles += 1;
    }
}

void CPU::BCC(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::CARRY)) == 0)
    {
        int32_t JumpDistance = Memory;
        if (JumpDistance > 127)
            JumpDistance -= 256;

        mRegisters.PC += JumpDistance;

        OpCode->Cycles += 1;
    }
}

void CPU::BCS(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::CARRY)) != 0)
    {
        int32_t JumpDistance = Memory;
        if (JumpDistance > 127)
            JumpDistance -= 256;

        mRegisters.PC += JumpDistance;

        OpCode->Cycles += 1;
    }
}

void CPU::BEQ(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::ZERO)) != 0)
    {
        int32_t JumpDistance = Memory;
        if (JumpDistance > 127)
            JumpDistance -= 256;

        mRegisters.PC = mRegisters.PC + JumpDistance;

        OpCode->Cycles += 1;
    }
}

void CPU::BVC(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::OVERFLOW)) == 0)
    {
        int32_t JumpDistance = Memory;
        if (JumpDistance > 127)
            JumpDistance -= 256;

        mRegisters.PC += JumpDistance;

        OpCode->Cycles += 1;
    }
}

void CPU::BVS(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::OVERFLOW)) != 0)
    {
        int32_t JumpDistance = Memory;
        if (JumpDistance > 127)
            JumpDistance -= 256;

        mRegisters.PC += JumpDistance;

        OpCode->Cycles += 1;
    }
}

void CPU::BIT(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    uint8_t A = mRegisters.A & Memory;

    if (A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (Memory & 0b01000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::OVERFLOW));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));

    if (Memory & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::BMI(NESOpCode* OpCode)
{
    uint8_t Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
    mRegisters.PC += 1;

    if ((mRegisters.P & static_cast<uint8_t>(EStatusFlags::NEGATIVE)) != 0)
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
    // BRK Pushes the next instruction address bytes to stack, along with Status Flags.
    mRegisters.PC += 2;

    uint8_t HighByte = (mRegisters.PC >> 8) & 0xFF;
    uint8_t LowByte = mRegisters.PC & 0xFF;

    PushStack(HighByte);
    PushStack(LowByte);

    // Only set on byte set on stack.
    auto CPUFlags = mRegisters.P;
    CPUFlags |= static_cast<uint8_t>(EStatusFlags::BFlag);

    PushStack(CPUFlags);

    mRegisters.P |= static_cast<uint8_t>(EStatusFlags::INTERRUPT_DISABLE);
    mRegisters.PC = 0xFFFE;
}

void CPU::JMP(const NESOpCode* OpCode)
{
    assert((OpCode->AddressMode == EAddressingMode::Absolute) || (OpCode->AddressMode == EAddressingMode::Indirect));

    if (OpCode->AddressMode == EAddressingMode::Absolute)
    {
        mRegisters.PC = mMemoryMapper->Read16Bit(mRegisters.PC);
    }
    else if (OpCode->AddressMode == EAddressingMode::Indirect)
    {
        // Copy pasted from ReadMemory function since it doesn't return 16 bits.
        uint8_t OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
        mRegisters.PC++;
        uint8_t OperandHighByte = mMemoryMapper->Read8Bit(mRegisters.PC);
        mRegisters.PC++;

        // 16 bit operand which points to another 16bit address which is the real target.
        uint16_t Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

        // In this step if the address ends in 0xFF it will read the wrong page.
        // Will need to emulate this later.
        OperandLowByte = mMemoryMapper->Read8Bit(Address);
        Address++;
        OperandHighByte = mMemoryMapper->Read8Bit(Address);

        Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

        mRegisters.PC = Address;
    }
}

void CPU::JSR()
{
    uint16_t SubroutineAddress = mMemoryMapper->Read16Bit(mRegisters.PC);
    mRegisters.PC += 1;

    PushStack(static_cast<uint8_t>((mRegisters.PC >> 8) & 0xFF));
    PushStack(static_cast<uint8_t>(mRegisters.PC & 0xFF));

    mRegisters.PC = SubroutineAddress;
}

void CPU::RTS()
{
    uint8_t LowByte = PopStack();
    uint8_t HighByte = PopStack();

    uint16_t ReturnAddress = static_cast<uint16_t>(HighByte) << 8;
    ReturnAddress |= LowByte;

    mRegisters.PC = ReturnAddress + 1;
}

void CPU::RTI()
{
    // Interrupt Disable from this pop is NOT delayed.
    mRegisters.P = PopStack();

    uint8_t LowByte = PopStack();
    uint8_t HighByte = PopStack();

    uint16_t ReturnAddress = static_cast<uint16_t>(HighByte) << 8;
    ReturnAddress |= LowByte;

    mRegisters.PC = ReturnAddress;
}

void CPU::CMP(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    int32_t A = int32_t(mRegisters.A) - Memory;

    if (A >= 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    if (A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::CPX(const NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    int32_t X = int32_t(mRegisters.X) - Memory;

    if (X >= 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    if (X == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (X & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::CPY(const NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    int32_t Y = int32_t(mRegisters.Y) - Memory;

    if (Y >= 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    if (Y == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (Y & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::SEI()
{
    // Should be delayed by one frame.
    mRegisters.P |= static_cast<uint8_t>((EStatusFlags::INTERRUPT_DISABLE));
}

void CPU::CLI()
{
    // Should be delayed by one frame.
    mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::INTERRUPT_DISABLE));
}

void CPU::CLD()
{
    mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::DECIMAL));
}

void CPU::CLC()
{
    mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));
}

void CPU::CLV()
{
    mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));
}

void CPU::SEC()
{
    mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
}

void CPU::SED()
{
    mRegisters.P |= static_cast<uint8_t>((EStatusFlags::DECIMAL));
}

void CPU::AND(const NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);
    mRegisters.A &= Memory;

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::ORA(const NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);
    mRegisters.A |= Memory;

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::ADC(const NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    bool Carry = (mRegisters.P & static_cast<uint8_t>((EStatusFlags::CARRY))) != 0;

    int32_t A = int32_t(mRegisters.A) + Memory + Carry;

    if (const bool bOverflowed = (~(mRegisters.A ^ Memory) & (mRegisters.A ^ A) & 0x80) != 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::OVERFLOW));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));

    if (A > 0xFF)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
         mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    mRegisters.A = uint8_t(A);

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::SBC(const NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    bool Carry = (mRegisters.P & static_cast<uint8_t>((EStatusFlags::CARRY))) != 0;

    int32_t A = int32_t(mRegisters.A)  + ~Memory + Carry;

    if (const bool bOverflowed = ((mRegisters.A ^ Memory) & (mRegisters.A ^ A) & 0x80) != 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::OVERFLOW));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));

    if (A >= 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    mRegisters.A = uint8_t(A);

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::DEC(const NESOpCode* OpCode)
{
    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    Memory -= 1;

    if (Memory== 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (Memory & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode->AddressMode, Memory);
}

void CPU::DEX()
{
    mRegisters.X -= 1;

    if (mRegisters.X == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.X & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::DEY()
{
    mRegisters.Y -= 1;

    if (mRegisters.Y == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.Y & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::EOR(const NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    mRegisters.A ^= Memory;

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::INC(const NESOpCode* OpCode)
{
    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode->AddressMode);

    Memory += 1;

    if (Memory == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (Memory & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode->AddressMode, Memory);
}

void CPU::INX()
{
    mRegisters.X += 1;

    if (mRegisters.X == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.X & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::INY()
{
    mRegisters.Y += 1;

    if (mRegisters.Y == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.Y & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::LDA(const NESOpCode* OpCode)
{
    mRegisters.A = ReadMemory(OpCode->AddressMode);

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::LDX(const NESOpCode* OpCode)
{
    mRegisters.X = ReadMemory(OpCode->AddressMode);

    if (mRegisters.X == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.X & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::LDY(const NESOpCode* OpCode)
{
    mRegisters.Y = ReadMemory(OpCode->AddressMode);

    if (mRegisters.Y == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.Y & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::PHA()
{
    PushStack(mRegisters.A);
}

void CPU::PHP()
{
    auto CPUFlags = mRegisters.P;
    CPUFlags |= static_cast<uint8_t>((EStatusFlags::BFlag));

    PushStack(CPUFlags);
}

void CPU::PLA()
{
    mRegisters.A = PopStack();

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::PLP()
{
    // TODO: Interrupt Disable effect should be delayed by one instruction.
    uint8_t CPUSTATUS = PopStack();

    // BFlag and ONEFLAG are ignored (Does this mean old state is kept?)
    CPUSTATUS &= ~static_cast<uint8_t>(EStatusFlags::BFlag);
    CPUSTATUS &= ~static_cast<uint8_t>(EStatusFlags::ONEFLAG);

    mRegisters.P = CPUSTATUS;
}

void CPU::STA(const NESOpCode* OpCode)
{
    WriteMemory(OpCode->AddressMode, mRegisters.A);
}

void CPU::STX(const NESOpCode* OpCode)
{
    WriteMemory(OpCode->AddressMode, mRegisters.X);
}

void CPU::STY(const NESOpCode* OpCode)
{
    WriteMemory(OpCode->AddressMode, mRegisters.Y);
}

void CPU::TAX()
{
    mRegisters.X = mRegisters.A;

    if (mRegisters.X == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.X & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::TAY()
{
    mRegisters.Y = mRegisters.A;

    if (mRegisters.Y == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.Y & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::TSX()
{
    mRegisters.X = mRegisters.S;

    if (mRegisters.X == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.X & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::TXS()
{
    mRegisters.S = mRegisters.X;
}

void CPU::TXA()
{
    mRegisters.A = mRegisters.X;

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::TYA()
{
    mRegisters.A = mRegisters.Y;

    if (mRegisters.A == 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::ZERO));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::ZERO));

    if (mRegisters.A & 0b10000000)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::NEGATIVE));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));
}

void CPU::NOP()
{
}
