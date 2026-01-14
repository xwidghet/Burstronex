#include "CPU.h"

#include "Logger.h"
#include "MemoryMapper.h"
#include "OpCodeDecoder.h"
#include "RomParameters.h"
#include "PPU.h"

#include <array>
#include <cassert>

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

    // Takes 7 cycles to initialize stack, and read the initial PC.
    mCycleCount = 7;

    DebugInit(ROM);

    mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::INFO, "Initial PC: {0:x}\n", mRegisters.PC);
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

double CPU::GetClockFrequency() const
{
    return mClockFrequency;
}

int64_t CPU::GetCycleCount() const
{
    return mCycleCount;
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

    mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "{0:04X} {1}   Registers: A:{2:02X}, X:{3:02X}, Y:{4:02X}, S:{5:02X}, P:{6:02X},   PCDATA:{7:02X}, Cycles: {8}\n", mRegisters.PC-1, OpCode.Name,
              mRegisters.A, mRegisters.X, mRegisters.Y, mRegisters.S, mRegisters.P, PCData, mCycleCount);

    ExecuteInstruction(&OpCode);
    mCycleCount += OpCode.Cycles;

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
            NOP(OpCode);
            break;
        case EINSTRUCTION::ALR:
            ALR(OpCode);
            break;
        case EINSTRUCTION::ANC:
            ANC(OpCode);
            break;
        case EINSTRUCTION::ARR:
            ARR(OpCode);
            break;
        case EINSTRUCTION::AXS:
            AXS(OpCode);
            break;
        case EINSTRUCTION::LAX:
            LAX(OpCode);
            break;
        case EINSTRUCTION::SAX:
            SAX(OpCode);
            break;
        case EINSTRUCTION::DCP:
            DCP(OpCode);
            break;
        case EINSTRUCTION::ISC:
            ISC(OpCode);
            break;
        case EINSTRUCTION::RLA:
            RLA(OpCode);
            break;
        case EINSTRUCTION::RRA:
            RRA(OpCode);
            break;
        case EINSTRUCTION::SLO:
            SLO(OpCode);
            break;
        case EINSTRUCTION::SRE:
            SRE(OpCode);
            break;
        default:
        {
            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::ERROR, "Unimplemented OpCode {0}\n", OpCode->Name);
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

void CPU::WriteMemory(NESOpCode* OpCode, uint8_t Value)
{
    // Not sure how to implement these, so asserting to find use cases.
    assert(OpCode->AddressMode != EAddressingMode::Implicit);
    assert(OpCode->AddressMode != EAddressingMode::Immediate);

    // Used for XIndexedIndirect which needs to retain the original read.
    // Should refactor the rest to be consistent.
    uint8_t Memory = 0;

    uint8_t OperandLowByte = 0;
    uint8_t OperandHighByte = 0;

    // Only used for...relative Addressing Mode.
    int32_t RelativeOperand = 0;

    uint16_t Address = 0;

    switch (OpCode->AddressMode)
    {
        // d,x
        case EAddressingMode::XZeroPageIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            // Wraps around 0x00 -> 0xFF
            Address = (int32_t(OperandLowByte) + mRegisters.X) & 0xFF;

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Read XZeroPageIndexed Address: {0:04X}\n", Address);

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
            Address = mMemoryMapper->Read16Bit(mRegisters.PC);
            mRegisters.PC += 2;

            Address += mRegisters.X;

            // Oops cycle
            OpCode->Cycles += 1;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // a,y
        case EAddressingMode::YAbsoluteIndexed:
            Address = mMemoryMapper->Read16Bit(mRegisters.PC);
            mRegisters.PC += 2;

            Address += mRegisters.Y;

            // Oops cycle
            OpCode->Cycles += 1;

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // (d, x)
        case EAddressingMode::XIndexedIndirect:
            Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (Memory + mRegisters.X) & 0xFF;
            OperandLowByte = mMemoryMapper->Read8Bit(Address);

            Address = (Memory + mRegisters.X + 1) & 0xFF;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Write XIndexedIndirect Address: {0:04X}\n", Address);

            mMemoryMapper->Write8Bit(Address, Value);
            break;
        // (d), y
        case EAddressingMode::YIndirectIndexed:
            Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = Memory & 0xFF;
            OperandLowByte = mMemoryMapper->Read8Bit(Address);

            Address = (Memory + 1) & 0xFF;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;
            Address += mRegisters.Y;

            // Oops write cycle
            OpCode->Cycles += 1;

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Write YIndirectIndexed Address: {0:04X}\n", Address);

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

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Write Zeropage Address: {0:04X}\n", Address);

            mMemoryMapper->Write8Bit(0x00 + OperandLowByte, Value);
            break;
        // a
        case EAddressingMode::Absolute:
            Address = mMemoryMapper->Read16Bit(mRegisters.PC);
            mRegisters.PC += 2;

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Write Absolute Address: {0:04X}\n", Address);

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
            // 16 bit operand which points to another 16bit address which is the real target.
            Address = mMemoryMapper->Read16Bit(mRegisters.PC);
            mRegisters.PC += 2;

            Address = mMemoryMapper->Read16Bit(Address);

            mMemoryMapper->Write8Bit(Address, Value);
            break;
    }
}

uint8_t CPU::ReadMemory(NESOpCode* OpCode)
{
    // NOP uses implicit
    //assert(OpCode->AddressMode != EAddressingMode::Implicit);

    // Used for XIndexedIndirect which needs to retain the original read.
    // Should refactor the rest to be consistent.
    uint8_t Memory = 0;

    uint8_t OperandLowByte = 0;
    uint8_t OperandHighByte = 0;

    // Only used for...relative Addressing Mode.
    int32_t RelativeOperand = 0;

    uint16_t Address = 0;

    switch (OpCode->AddressMode)
    {
        // d,x
        case EAddressingMode::XZeroPageIndexed:
            OperandLowByte = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            // Wraps around 0x00 -> 0xFF
            Address = (int32_t(OperandLowByte) + mRegisters.X) & 0xFF;

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Read XZeroPageIndexed Address: {0:04X}\n", Address);

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
            Address = mMemoryMapper->Read16Bit(mRegisters.PC);
            mRegisters.PC += 2;

            Address += mRegisters.X;

            // Page wrap oops cycle.
            OpCode->Cycles += PageCrossed(Address, Address - mRegisters.X);

            return mMemoryMapper->Read8Bit(Address);
            break;
            // a,y
        case EAddressingMode::YAbsoluteIndexed:
            Address = mMemoryMapper->Read16Bit(mRegisters.PC);
            mRegisters.PC += 2;

            Address += mRegisters.Y;

            // Page wrap oops cycle.
            OpCode->Cycles += PageCrossed(Address, Address - mRegisters.Y);

            return mMemoryMapper->Read8Bit(Address);
            break;
            // (d, x)
        case EAddressingMode::XIndexedIndirect:
            Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = (Memory + mRegisters.X) & 0xFF;
            OperandLowByte = mMemoryMapper->Read8Bit(Address);

            Address = (Memory + mRegisters.X + 1) & 0xFF;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Read XIndexedIndirect Address: {0:04X}\n", Address);

            return mMemoryMapper->Read8Bit(Address);
            break;
            // (d), y
        case EAddressingMode::YIndirectIndexed:
            Memory = mMemoryMapper->Read8Bit(mRegisters.PC);
            mRegisters.PC++;

            Address = Memory & 0xFF;
            OperandLowByte = mMemoryMapper->Read8Bit(Address);

            Address = (Memory + 1) & 0xFF;
            OperandHighByte = mMemoryMapper->Read8Bit(Address);

            Address = (static_cast<uint16_t>(OperandHighByte) << 8) | OperandLowByte;
            Address += mRegisters.Y;

            OpCode->Cycles += PageCrossed(Address, Address - mRegisters.Y);

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Read YIndirectIndexed Address: {0:04X}\n", Address);

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

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Read Zeropage Address: {0:04X}\n", Address);

            return mMemoryMapper->Read8Bit(Address);
            break;
            // a
        case EAddressingMode::Absolute:
            Address = mMemoryMapper->Read16Bit(mRegisters.PC);
            mRegisters.PC += 2;

            mLog->Log(ELOGGING_SOURCES::CPU, ELOGGING_MODE::VERBOSE, "Read Absolute Address: {0:X}, {1:02X}\n", Address, mMemoryMapper->Read8Bit(Address));

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
            // 16 bit operand which points to another 16bit address which is the real target.
            Address = mMemoryMapper->Read16Bit(mRegisters.PC);
            mRegisters.PC += 2;

            Address = mMemoryMapper->Read16Bit(Address);

            return mMemoryMapper->Read8Bit(Address);
            break;
    }

    // Should Never Happen TM
    return 0;
}

bool CPU::PageCrossed(const uint16_t Left, const uint16_t Right)
{
    return (Left & 0xFF00) != (Right & 0xFF00);
}

void CPU::SetStatusBit(const EStatusFlags StatusFlag, const bool Enabled)
{
    uint8_t StatusMask = static_cast<uint8_t>(StatusFlag);

    // Branchless way to set a bit to 0 or 1.
    mRegisters.P &= ~StatusMask;
    mRegisters.P |= (uint8_t(0x00 - Enabled) & StatusMask);
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
    auto OriginalCycleCount = OpCode->Cycles;

    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode);
    bool Carry = (Memory & 0b10000000) != 0;

    Memory = Memory << 1;

    SetStatusBit(EStatusFlags::CARRY, Carry);
    SetStatusBit(EStatusFlags::ZERO, Memory == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, Memory >> 7);

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode, Memory);

    // LSR, ASL, ROL, ROR do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
}

void CPU::LSR(NESOpCode* OpCode)
{
    auto OriginalCycleCount = OpCode->Cycles;

    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode);
    bool Carry = (Memory & 0b00000001) != 0;

    Memory = Memory >> 1;

    SetStatusBit(EStatusFlags::CARRY, Carry);
    SetStatusBit(EStatusFlags::ZERO, Memory == 0);

    // Since Bit 7 is always zero after RHS, this is always set to zero.
    mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::NEGATIVE));

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode, Memory);


    // LSR, ASL, ROL, ROR do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
}

void CPU::ROL(NESOpCode* OpCode)
{
    auto OriginalCycleCount = OpCode->Cycles;

    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode);
    bool MemoryCarry = (Memory & 0b10000000) != 0;
    bool CPUCarry = (mRegisters.P & static_cast<uint8_t>(EStatusFlags::CARRY)) != 0;

    Memory = Memory << 1;

    Memory |= static_cast<uint8_t>(CPUCarry);

    SetStatusBit(EStatusFlags::CARRY, MemoryCarry);
    SetStatusBit(EStatusFlags::ZERO, Memory == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, Memory >> 7);

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode, Memory);

    // LSR, ASL, ROL, ROR do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
}

void CPU::ROR(NESOpCode* OpCode)
{
    auto OriginalCycleCount = OpCode->Cycles;

    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode);
    bool MemoryCarry = (Memory & 0b00000001) != 0;
    bool CPUCarry = (mRegisters.P & static_cast<uint8_t>(EStatusFlags::CARRY)) != 0;

    Memory = Memory >> 1;

    Memory |= (static_cast<uint8_t>(CPUCarry) << 7);

    SetStatusBit(EStatusFlags::CARRY, MemoryCarry);
    SetStatusBit(EStatusFlags::ZERO, Memory == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, Memory >> 7);

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode, Memory);

    // LSR, ASL, ROL, ROR do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
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

        uint16_t OldPC = mRegisters.PC;
        mRegisters.PC += JumpDistance;

        if (PageCrossed(OldPC,mRegisters.PC))
            OpCode->Cycles += 1;

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

        uint16_t OldPC = mRegisters.PC;
        mRegisters.PC += JumpDistance;

        if (PageCrossed(OldPC,mRegisters.PC))
            OpCode->Cycles += 1;

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

        uint16_t OldPC = mRegisters.PC;
        mRegisters.PC += JumpDistance;

        if (PageCrossed(OldPC,mRegisters.PC))
            OpCode->Cycles += 1;

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

        uint16_t OldPC = mRegisters.PC;
        mRegisters.PC += JumpDistance;

        if (PageCrossed(OldPC,mRegisters.PC))
            OpCode->Cycles += 1;

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

        uint16_t OldPC = mRegisters.PC;
        mRegisters.PC += JumpDistance;

        if (PageCrossed(OldPC,mRegisters.PC))
            OpCode->Cycles += 1;

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

        uint16_t OldPC = mRegisters.PC;
        mRegisters.PC += JumpDistance;

        if (PageCrossed(OldPC,mRegisters.PC))
            OpCode->Cycles += 1;

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

        uint16_t OldPC = mRegisters.PC;
        mRegisters.PC += JumpDistance;

        if (PageCrossed(OldPC,mRegisters.PC))
            OpCode->Cycles += 1;

        OpCode->Cycles += 1;
    }
}

void CPU::BIT(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);

    uint8_t A = mRegisters.A & Memory;

    SetStatusBit(EStatusFlags::ZERO, A == 0);
    SetStatusBit(EStatusFlags::OVERFLOW, (Memory >> 6) & 0b1);
    SetStatusBit(EStatusFlags::NEGATIVE, Memory >> 7);
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

        uint16_t OldPC = mRegisters.PC;
        mRegisters.PC += JumpDistance;

        if (PageCrossed(OldPC,mRegisters.PC))
            OpCode->Cycles += 1;

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

void CPU::JMP(NESOpCode* OpCode)
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

        OperandLowByte = mMemoryMapper->Read8Bit(Address);
        Address++;

        // CPU Bug: If the address crosses a page, it still reads from the previous page
        //          Ex. First Byte Address 0x03FF
        //              Second Byte Address 0x0300 (Not 0x0400)
        if ((Address & 0xFF) == 0)
        {
            Address -= 0x100;
        }

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
    uint8_t Memory = ReadMemory(OpCode);

    int32_t A = int32_t(mRegisters.A) - Memory;

    SetStatusBit(EStatusFlags::CARRY, A >= 0);
    SetStatusBit(EStatusFlags::ZERO, A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, uint8_t(A) >> 7);

}

void CPU::CPX(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);

    int32_t X = int32_t(mRegisters.X) - Memory;

    SetStatusBit(EStatusFlags::CARRY, X >= 0);
    SetStatusBit(EStatusFlags::ZERO, X == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, uint8_t(X) >> 7);
}

void CPU::CPY(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);

    int32_t Y = int32_t(mRegisters.Y) - Memory;

    SetStatusBit(EStatusFlags::CARRY, Y >= 0);
    SetStatusBit(EStatusFlags::ZERO, Y == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, uint8_t(Y) >> 7);
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

void CPU::AND(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);
    mRegisters.A &= Memory;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
}

void CPU::ORA(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);
    mRegisters.A |= Memory;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
}

void CPU::ADC(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);

    uint32_t Carry = (mRegisters.P & static_cast<uint8_t>((EStatusFlags::CARRY))) != 0;

    uint32_t Temp = uint32_t(mRegisters.A) + Memory + Carry;

    if ((Temp ^ mRegisters.A) & (Temp ^ Memory) & 0x80)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::OVERFLOW));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));

    if ((Temp >> 8) & 1)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
         mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    mRegisters.A = uint8_t(Temp & 0xFF);

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
}

void CPU::SBC(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);
    Memory ^= 0xFF;

    uint32_t Carry = (mRegisters.P & static_cast<uint8_t>((EStatusFlags::CARRY))) != 0;

    uint32_t Temp = uint32_t(mRegisters.A)  + Memory + Carry;

    if ((Temp ^ mRegisters.A) & (Temp ^ Memory) & 0x80)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::OVERFLOW));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));

    if ((Temp >> 8) & 1)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    mRegisters.A = uint8_t(Temp & 0xFF);

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
}

void CPU::DEC(NESOpCode* OpCode)
{
    auto OriginalCycleCount = OpCode->Cycles;

    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode);

    Memory -= 1;

    SetStatusBit(EStatusFlags::ZERO, Memory == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, Memory >> 7);

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode, Memory);

    // INC, DEC, do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
}

void CPU::DEX()
{
    mRegisters.X -= 1;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.X == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.X >> 7);
}

void CPU::DEY()
{
    mRegisters.Y -= 1;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.Y == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.Y >> 7);
}

void CPU::EOR(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);

    mRegisters.A ^= Memory;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
}

void CPU::INC(NESOpCode* OpCode)
{
    auto OriginalCycleCount = OpCode->Cycles;

    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    uint8_t Memory = ReadMemory(OpCode);

    Memory += 1;

    SetStatusBit(EStatusFlags::ZERO, Memory == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, Memory >> 7);

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode, Memory);

    // INC, DEC, do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
}

void CPU::INX()
{
    mRegisters.X += 1;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.X == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.X >> 7);
}

void CPU::INY()
{
    mRegisters.Y += 1;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.Y == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.Y >> 7);
}

void CPU::LDA(NESOpCode* OpCode)
{
    mRegisters.A = ReadMemory(OpCode);

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
}

void CPU::LDX(NESOpCode* OpCode)
{
    mRegisters.X = ReadMemory(OpCode);

    SetStatusBit(EStatusFlags::ZERO, mRegisters.X == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.X >> 7);
}

void CPU::LDY(NESOpCode* OpCode)
{
    mRegisters.Y = ReadMemory(OpCode);

    SetStatusBit(EStatusFlags::ZERO, mRegisters.Y == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.Y >> 7);
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

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
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

void CPU::STA(NESOpCode* OpCode)
{
    auto OriginalCycleCount = OpCode->Cycles;
    WriteMemory(OpCode, mRegisters.A);

    // STA, STX, STY do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
}

void CPU::STX(NESOpCode* OpCode)
{
    auto OriginalCycleCount = OpCode->Cycles;
    WriteMemory(OpCode, mRegisters.X);

    // STA, STX, STY do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
}

void CPU::STY(NESOpCode* OpCode)
{
    auto OriginalCycleCount = OpCode->Cycles;
    WriteMemory(OpCode, mRegisters.Y);

    // STA, STX, STY do not consume more cycles on page miss.
    OpCode->Cycles = OriginalCycleCount;
}

void CPU::TAX()
{
    mRegisters.X = mRegisters.A;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.X == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.X >> 7);
}

void CPU::TAY()
{
    mRegisters.Y = mRegisters.A;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.Y == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.Y >> 7);
}

void CPU::TSX()
{
    mRegisters.X = mRegisters.S;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.X == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.X >> 7);
}

void CPU::TXS()
{
    mRegisters.S = mRegisters.X;
}

void CPU::TXA()
{
    mRegisters.A = mRegisters.X;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
}

void CPU::TYA()
{
    mRegisters.A = mRegisters.Y;

    SetStatusBit(EStatusFlags::ZERO, mRegisters.A == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.A >> 7);
}

void CPU::NOP(NESOpCode* OpCode)
{
    ReadMemory(OpCode);
}

void CPU::ALR(NESOpCode* OpCode)
{
    auto OldPC = mRegisters.PC;
    OpCode->AddressMode = EAddressingMode::Immediate;
    AND(OpCode);

    mRegisters.PC = OldPC;
    OpCode->AddressMode = EAddressingMode::Accumulator;
    LSR(OpCode);

    // Immediate increment
    mRegisters.PC++;
}

void CPU::ANC(NESOpCode* OpCode)
{
    OpCode->AddressMode = EAddressingMode::Immediate;
    AND(OpCode);

    if (mRegisters.P & static_cast<uint8_t>((EStatusFlags::NEGATIVE)))
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));
}

void CPU::ARR(NESOpCode* OpCode)
{
    auto OldPC = mRegisters.PC;
    OpCode->AddressMode = EAddressingMode::Immediate;
    AND(OpCode);
    mRegisters.PC = OldPC;

    // HACK FOR NOT RETURNING MEMORY POINTERS...NOT TOO SURE THE RIGHT WAY TO ARCHITECT THIS
    auto TargetPC = mRegisters.PC;

    // ROR Copy, with modifications
    OpCode->AddressMode = EAddressingMode::Accumulator;
    uint8_t Memory = ReadMemory(OpCode);
    bool MemoryCarry = (Memory & 0b00000001) != 0;
    bool CPUCarry = (mRegisters.P & static_cast<uint8_t>(EStatusFlags::CARRY)) != 0;

    Memory = Memory >> 1;

    Memory |= (static_cast<uint8_t>(CPUCarry) << 7);

    bool bBit5 = Memory & 0b00100000;
    bool bBit6 = Memory & 0b01000000;

    // Carry is bit 6? Or should this be the original bit 6?
    if (bBit6)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    // Register V (Overflow)
    if (bBit6 ^ bBit5)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::OVERFLOW));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::OVERFLOW));

    SetStatusBit(EStatusFlags::ZERO, Memory == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, Memory >> 7);

    mRegisters.PC = TargetPC;
    WriteMemory(OpCode, Memory);
}

void CPU::AXS(NESOpCode* OpCode)
{
    uint8_t Memory = ReadMemory(OpCode);
    mRegisters.X &= mRegisters.A;

    // AXS doesn't borrow, and doesn't write Overflow
    int32_t X = int32_t(mRegisters.X) + ~Memory;

    if (X >= 0)
        mRegisters.P |= static_cast<uint8_t>((EStatusFlags::CARRY));
    else
        mRegisters.P &= ~static_cast<uint8_t>((EStatusFlags::CARRY));

    mRegisters.X = uint8_t(X);

    SetStatusBit(EStatusFlags::ZERO, mRegisters.X == 0);
    SetStatusBit(EStatusFlags::NEGATIVE, mRegisters.X >> 7);
}

void CPU::LAX(NESOpCode* OpCode)
{
    LDA(OpCode);
    TAX();
}

void CPU::SAX(NESOpCode* OpCode)
{
    WriteMemory(OpCode, mRegisters.A & mRegisters.X);
}

void CPU::DCP(NESOpCode* OpCode)
{
    auto OldPC = mRegisters.PC;
    DEC(OpCode);

    mRegisters.PC = OldPC;
    CMP(OpCode);
}

void CPU::ISC(NESOpCode* OpCode)
{
    auto OldPC = mRegisters.PC;
    INC(OpCode);

    mRegisters.PC = OldPC;
    SBC(OpCode);
}

void CPU::RLA(NESOpCode* OpCode)
{
    auto OldPC = mRegisters.PC;
    ROL(OpCode);

    mRegisters.PC = OldPC;
    AND(OpCode);
}

void CPU::RRA(NESOpCode* OpCode)
{
    auto OldPC = mRegisters.PC;
    ROR(OpCode);

    mRegisters.PC = OldPC;
    ADC(OpCode);
}

void CPU::SLO(NESOpCode* OpCode)
{
    auto OldPC = mRegisters.PC;
    ASL(OpCode);

    mRegisters.PC = OldPC;
    ORA(OpCode);
}

void CPU::SRE(NESOpCode* OpCode)
{
    auto OldPC = mRegisters.PC;
    LSR(OpCode);

    mRegisters.PC = OldPC;
    EOR(OpCode);
}
