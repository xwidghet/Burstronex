#pragma once

#include <array>
#include <cstdint>
#include <string>

enum class EAddressingMode
{
    // d,x
    XZeroPageIndexed,
    // d,y
    YZeroPageIndexed,
    // a,x
    XAbsoluteIndexed,
    // a,y
    YAbsoluteIndexed,
    // (d, x)
    XIndexedIndirect,
    // (d), y
    YIndirectIndexed,
    // No address operand, ex. RTS / CLC
    Implicit,
    // A
    Accumulator,
    // #i (Wiki called it #v for some unknown reason????)
    Immediate,
    // d
    Zeropage,
    // a
    Absolute,
    // *+d (label??)
    Relative,
    // (a)
    Indirect
};

// Official Op Codes first, Unofficial second
enum class EINSTRUCTION
{
  ADC, AND, ASL, BCC, BCS, BEQ, BIT, BMI, BNE, BPL, BRK, BVC, BVS, CLC,
  CLD, CLI, CLV, CMP, CPX, CPY, DEC, DEX, DEY, EOR, INC, INX, INY, JMP,
  JSR, LDA, LDX, LDY, LSR, NOP, ORA, PHA, PHP, PLA, PLP, ROL, ROR, RTI,
  RTS, SBC, SEC, SED, SEI, STA, STX, STY, TAX, TAY, TSX, TXA, TXS, TYA,

  SLO, ANC, RLA, SRE, ALR, RRA, ARR, SAX, XAA, AHX, TAS, LAX, LAS, DCP,
  AXS, ISC, SHY, STP, SHX
};

struct NESOpCode {
  std::string Name;
  EINSTRUCTION Instruction;
  EAddressingMode AddressMode;
  uint8_t OperandByteCount;
  uint8_t Cycles;
};

class OpCodeDecoder {
public:
    static NESOpCode DecodeOpCode(const uint8_t PCData);
};
