#include "OpCodeDecoder.h"

// How many braces could a wood chuck chuck if a wood chuck could chuck wood.
// Cycle counts are the constant cost, branches add one cycle, crossing memory pages adds one cycle.
// Sizes are Instruction + Operands,
// Ex. Immediate 1 Instruction + 1 Operand, Absolute Indexed 1 instruction + 2 operand.
std::array<std::array<NESOpCode, 32>, 8> OpCodeTable =
{{
    // 00
    {{
        // Red Block
        {  "BRK", EINSTRUCTION::BRK, EAddressingMode::Implicit, 2, 7},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Zeropage, 1, 2},
        {  "PHP", EINSTRUCTION::PHP, EAddressingMode::Implicit, 1, 3},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Absolute, 1, 2},
        {  "BPL", EINSTRUCTION::BPL, EAddressingMode::Relative, 2, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XZeroPageIndexed, 1, 2},
        {  "CLC", EINSTRUCTION::CLC, EAddressingMode::Implicit, 1, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XAbsoluteIndexed, 1, 2},

        // Green Block
        {  "ORA", EINSTRUCTION::ORA, EAddressingMode::XIndexedIndirect, 2, 6},
        {  "ORA", EINSTRUCTION::ORA, EAddressingMode::Zeropage, 2, 3},
        {  "ORA", EINSTRUCTION::ORA, EAddressingMode::Immediate, 2, 2},
        {  "ORA", EINSTRUCTION::ORA, EAddressingMode::Absolute, 3, 4},
        {  "ORA", EINSTRUCTION::ORA, EAddressingMode::YIndirectIndexed, 2, 5},
        {  "ORA", EINSTRUCTION::ORA, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "ORA", EINSTRUCTION::ORA, EAddressingMode::YAbsoluteIndexed, 3, 4},
        {  "ORA", EINSTRUCTION::ORA, EAddressingMode::XAbsoluteIndexed, 3, 4},

        // Blue Block (UPDATE ADDRESS MODE AND CYCLES AND BYTES)
        // STP is the Stop instruction, no documentation and generally unused.
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "ASL", EINSTRUCTION::ASL, EAddressingMode::Zeropage, 2, 5},
        {  "ASL", EINSTRUCTION::ASL, EAddressingMode::Accumulator, 1, 2},
        {  "ASL", EINSTRUCTION::ASL, EAddressingMode::Absolute, 3, 6},
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "ASL", EINSTRUCTION::ASL, EAddressingMode::XZeroPageIndexed, 2, 6},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Implicit, 1, 2},
        {  "ASL", EINSTRUCTION::ASL, EAddressingMode::XAbsoluteIndexed, 3, 7},

        // Grey Block (Unknown Operand Sizes and Cycle Counts)
        {  "SLO", EINSTRUCTION::SLO, EAddressingMode::XIndexedIndirect, 2, 255},
        {  "SLO", EINSTRUCTION::SLO, EAddressingMode::Zeropage, 2, 255},
        {  "ANC", EINSTRUCTION::ANC, EAddressingMode::Immediate, 2, 255},
        {  "SLO", EINSTRUCTION::SLO, EAddressingMode::Absolute, 3, 255},
        {  "SLO", EINSTRUCTION::SLO, EAddressingMode::YIndirectIndexed, 2, 255},
        {  "SLO", EINSTRUCTION::SLO, EAddressingMode::XZeroPageIndexed, 2, 255},
        {  "SLO", EINSTRUCTION::SLO, EAddressingMode::YAbsoluteIndexed, 3, 255},
        {  "SLO", EINSTRUCTION::SLO, EAddressingMode::XAbsoluteIndexed, 3, 255}
    }},
    // 20
    {{
        // Red Block
        {  "JSR", EINSTRUCTION::JSR, EAddressingMode::Absolute, 3, 6},
        {  "BIT", EINSTRUCTION::BIT, EAddressingMode::Zeropage, 2, 3},
        {  "PLP", EINSTRUCTION::PLP, EAddressingMode::Implicit, 1, 4},
        {  "BIT", EINSTRUCTION::BIT, EAddressingMode::Absolute, 3, 4},
        {  "BMI", EINSTRUCTION::BMI, EAddressingMode::Zeropage, 2, 3},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XZeroPageIndexed, 1, 2},
        {  "SEC", EINSTRUCTION::SEC, EAddressingMode::Implicit, 1, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XAbsoluteIndexed, 1, 2},

        // Green Block
        {  "AND", EINSTRUCTION::AND, EAddressingMode::XIndexedIndirect, 2, 6},
        {  "AND", EINSTRUCTION::AND, EAddressingMode::Zeropage, 2, 3},
        {  "AND", EINSTRUCTION::AND, EAddressingMode::Immediate, 2, 2},
        {  "AND", EINSTRUCTION::AND, EAddressingMode::Absolute, 3, 4},
        {  "AND", EINSTRUCTION::AND, EAddressingMode::YIndirectIndexed, 2, 5},
        {  "AND", EINSTRUCTION::AND, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "AND", EINSTRUCTION::AND, EAddressingMode::YAbsoluteIndexed, 3, 4},
        {  "AND", EINSTRUCTION::AND, EAddressingMode::XAbsoluteIndexed, 3, 4},

        // Blue Block
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "ROL", EINSTRUCTION::ROL, EAddressingMode::Zeropage, 2, 5},
        {  "ROL", EINSTRUCTION::ROL, EAddressingMode::Accumulator, 1, 2},
        {  "ROL", EINSTRUCTION::ROL, EAddressingMode::Absolute, 3, 6},
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "ROL", EINSTRUCTION::ROL, EAddressingMode::XZeroPageIndexed, 2, 6},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Implicit, 1, 2},
        {  "ROL", EINSTRUCTION::ROL, EAddressingMode::XAbsoluteIndexed, 3, 7},

        // Grey Block (Unkown Sizes and Cycle Counts, supposedly Addressing Mode gives size)
        {  "RLA", EINSTRUCTION::RLA, EAddressingMode::XIndexedIndirect, 3, 7},
        {  "RLA", EINSTRUCTION::RLA, EAddressingMode::Zeropage, 2, 7},
        {  "ANC", EINSTRUCTION::ANC, EAddressingMode::Immediate, 2, 7},
        {  "RLA", EINSTRUCTION::RLA, EAddressingMode::Absolute, 3, 7},
        {  "RLA", EINSTRUCTION::RLA, EAddressingMode::YIndirectIndexed, 2, 7},
        {  "RLA", EINSTRUCTION::RLA, EAddressingMode::XZeroPageIndexed, 2, 7},
        {  "RLA", EINSTRUCTION::RLA, EAddressingMode::YAbsoluteIndexed, 3, 7},
        {  "RLA", EINSTRUCTION::RLA, EAddressingMode::XAbsoluteIndexed, 3, 7}
    }},
    // 40
    {{
        // Red Block
        {  "RTI", EINSTRUCTION::RTI, EAddressingMode::Implicit, 1, 6},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Zeropage, 1, 2},
        {  "PHA", EINSTRUCTION::PHA, EAddressingMode::Implicit, 1, 3},
        {  "JMP", EINSTRUCTION::JMP, EAddressingMode::Absolute, 3, 3},
        {  "BVC", EINSTRUCTION::BVC, EAddressingMode::Relative, 2, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XZeroPageIndexed, 1, 2},
        {  "CLI", EINSTRUCTION::CLI, EAddressingMode::Implicit, 1, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XAbsoluteIndexed, 1, 2},

        // Green Block
        {  "EOR", EINSTRUCTION::EOR, EAddressingMode::XIndexedIndirect, 2, 6},
        {  "EOR", EINSTRUCTION::EOR, EAddressingMode::Zeropage, 2, 3},
        {  "EOR", EINSTRUCTION::EOR, EAddressingMode::Immediate, 2, 2},
        {  "EOR", EINSTRUCTION::EOR, EAddressingMode::Absolute, 3, 4},
        {  "EOR", EINSTRUCTION::EOR, EAddressingMode::YIndirectIndexed, 2, 5},
        {  "EOR", EINSTRUCTION::EOR, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "EOR", EINSTRUCTION::EOR, EAddressingMode::YAbsoluteIndexed, 3, 4},
        {  "EOR", EINSTRUCTION::EOR, EAddressingMode::XAbsoluteIndexed, 3, 4},

        // Blue Block
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "LSR", EINSTRUCTION::LSR, EAddressingMode::Zeropage, 2, 5},
        {  "LSR", EINSTRUCTION::LSR, EAddressingMode::Accumulator, 1, 2},
        {  "LSR", EINSTRUCTION::LSR, EAddressingMode::Absolute, 3, 6},
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "LSR", EINSTRUCTION::LSR, EAddressingMode::XZeroPageIndexed, 2, 6},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Implicit, 1, 2},
        {  "LSR", EINSTRUCTION::LSR, EAddressingMode::XAbsoluteIndexed, 3, 7},

        // Grey Block
        {  "SRE", EINSTRUCTION::SRE, EAddressingMode::XIndexedIndirect, 2, 255},
        {  "SRE", EINSTRUCTION::SRE, EAddressingMode::Zeropage, 2, 255},
        {  "ALR", EINSTRUCTION::ALR, EAddressingMode::Immediate, 2, 255},
        {  "SRE", EINSTRUCTION::SRE, EAddressingMode::Absolute, 3, 255},
        {  "SRE", EINSTRUCTION::SRE, EAddressingMode::YIndirectIndexed, 2, 255},
        {  "SRE", EINSTRUCTION::SRE, EAddressingMode::XZeroPageIndexed, 2, 255},
        {  "SRE", EINSTRUCTION::SRE, EAddressingMode::YAbsoluteIndexed, 3, 255},
        {  "SRE", EINSTRUCTION::SRE, EAddressingMode::XAbsoluteIndexed, 3, 255}
    }},
    // 60
    {{
        // Red Block
        {  "RTS", EINSTRUCTION::RTS, EAddressingMode::Implicit, 1, 6},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Zeropage, 1, 2},
        {  "PLA", EINSTRUCTION::PLA, EAddressingMode::Implicit, 1, 4},
        {  "JMP", EINSTRUCTION::JMP, EAddressingMode::Indirect, 3, 5},
        {  "BVS", EINSTRUCTION::BVS, EAddressingMode::Relative, 2, 3},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XZeroPageIndexed, 1, 2},
        {  "SEI", EINSTRUCTION::SEI, EAddressingMode::Implicit, 1, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XAbsoluteIndexed, 1, 2},

        // Green Block
        { "ADC", EINSTRUCTION::ADC, EAddressingMode::XZeroPageIndexed, 2, 4},
        { "ADC", EINSTRUCTION::ADC, EAddressingMode::Zeropage, 2, 3},
        { "ADC", EINSTRUCTION::ADC, EAddressingMode::Immediate, 2, 2},
        { "ADC", EINSTRUCTION::ADC, EAddressingMode::Absolute, 3, 4},
        { "ADC", EINSTRUCTION::ADC, EAddressingMode::YIndirectIndexed, 2, 5},
        { "ADC", EINSTRUCTION::ADC, EAddressingMode::XZeroPageIndexed, 3, 4},
        { "ADC", EINSTRUCTION::ADC, EAddressingMode::YAbsoluteIndexed, 3, 4},
        { "ADC", EINSTRUCTION::ADC, EAddressingMode::XAbsoluteIndexed, 3, 4},

        // Blue Block
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "ROR", EINSTRUCTION::ROR, EAddressingMode::Zeropage, 2, 5},
        {  "ROR", EINSTRUCTION::ROR, EAddressingMode::Accumulator, 1, 2},
        {  "ROR", EINSTRUCTION::ROR, EAddressingMode::Absolute, 3, 6},
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "ROR", EINSTRUCTION::ROR, EAddressingMode::XZeroPageIndexed, 2, 6},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Implicit, 1, 2},
        {  "ROR", EINSTRUCTION::ROR, EAddressingMode::XAbsoluteIndexed, 3, 7},

        // Grey Block
        {  "RRA", EINSTRUCTION::RRA, EAddressingMode::XIndexedIndirect, 2, 255},
        {  "RRA", EINSTRUCTION::RRA, EAddressingMode::Zeropage, 2, 255},
        {  "ARR", EINSTRUCTION::ARR, EAddressingMode::Immediate, 2, 255},
        {  "RRA", EINSTRUCTION::RRA, EAddressingMode::Absolute, 3, 255},
        {  "RRA", EINSTRUCTION::RRA, EAddressingMode::YIndirectIndexed, 2, 255},
        {  "RRA", EINSTRUCTION::RRA, EAddressingMode::XZeroPageIndexed, 3, 255},
        {  "RRA", EINSTRUCTION::RRA, EAddressingMode::YAbsoluteIndexed, 3, 255},
        {  "RRA", EINSTRUCTION::RRA, EAddressingMode::XAbsoluteIndexed, 3, 255}
    }},
    // 80
    {{
        // Red Block
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Immediate, 1, 2},
        {  "STY", EINSTRUCTION::STY, EAddressingMode::Zeropage, 2, 3},
        {  "DEY", EINSTRUCTION::DEY, EAddressingMode::Implicit, 1, 2},
        {  "STY", EINSTRUCTION::STY, EAddressingMode::Absolute, 3, 4},
        {  "BCC", EINSTRUCTION::BCC, EAddressingMode::Relative, 2, 3},
        {  "STY", EINSTRUCTION::STY, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "TYA", EINSTRUCTION::TYA, EAddressingMode::Implicit, 1, 2},
        {  "SHY", EINSTRUCTION::SHY, EAddressingMode::XAbsoluteIndexed, 2, 5},

        // Green Block
        {  "STA", EINSTRUCTION::STA, EAddressingMode::XIndexedIndirect, 2, 6},
        {  "STA", EINSTRUCTION::STA, EAddressingMode::Zeropage, 2, 3},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Immediate, 1, 2},
        {  "STA", EINSTRUCTION::STA, EAddressingMode::Absolute, 3, 4},
        {  "STA", EINSTRUCTION::STA, EAddressingMode::YIndirectIndexed, 2, 6},
        {  "STA", EINSTRUCTION::STA, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "STA", EINSTRUCTION::STA, EAddressingMode::YAbsoluteIndexed, 3, 5},
        {  "STA", EINSTRUCTION::STA, EAddressingMode::XAbsoluteIndexed, 3, 5},

        // Blue Block
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Immediate, 1, 2},
        {  "STX", EINSTRUCTION::STX, EAddressingMode::Zeropage, 2, 3},
        {  "TXA", EINSTRUCTION::TXA, EAddressingMode::Implicit, 1, 2},
        {  "STX", EINSTRUCTION::STX, EAddressingMode::Absolute, 3, 4},
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "STX", EINSTRUCTION::STX, EAddressingMode::YZeroPageIndexed, 2, 4},
        {  "TXS", EINSTRUCTION::TXS, EAddressingMode::Implicit, 1, 2},
        {  "SHX", EINSTRUCTION::SHX, EAddressingMode::YAbsoluteIndexed, 1, 5},

        // Grey Block
        {  "SAX", EINSTRUCTION::SAX, EAddressingMode::XIndexedIndirect, 2, 255},
        {  "SAX", EINSTRUCTION::SAX, EAddressingMode::Zeropage, 2, 255},
        {  "XAA", EINSTRUCTION::XAA, EAddressingMode::Immediate, 1, 255},
        {  "SAX", EINSTRUCTION::SAX, EAddressingMode::Absolute, 3, 255},
        {  "AHX", EINSTRUCTION::AHX, EAddressingMode::YIndirectIndexed, 1, 6},
        {  "SAX", EINSTRUCTION::SAX, EAddressingMode::YZeroPageIndexed, 2, 255},
        {  "TAS", EINSTRUCTION::TAS, EAddressingMode::YAbsoluteIndexed, 1, 5},
        {  "AHX", EINSTRUCTION::AHX, EAddressingMode::YAbsoluteIndexed, 3, 6}
    }},
    // A0
    {{
        // Red Block
        {  "LDY", EINSTRUCTION::LDY, EAddressingMode::Immediate, 2, 2},
        {  "LDY", EINSTRUCTION::LDY, EAddressingMode::Zeropage, 2, 3},
        {  "TAY", EINSTRUCTION::TAY, EAddressingMode::Implicit, 1, 2},
        {  "LDY", EINSTRUCTION::LDY, EAddressingMode::Absolute, 3, 4},
        {  "BCS", EINSTRUCTION::BCS, EAddressingMode::Relative, 2, 3},
        {  "LDY", EINSTRUCTION::LDY, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "CLV", EINSTRUCTION::CLV, EAddressingMode::Implicit, 1, 2},
        {  "LDY", EINSTRUCTION::LDY, EAddressingMode::XAbsoluteIndexed, 3, 4},

        // Green Block
        {  "LDA", EINSTRUCTION::LDA, EAddressingMode::XIndexedIndirect, 2, 6},
        {  "LDA", EINSTRUCTION::LDA, EAddressingMode::Zeropage, 2, 3},
        {  "LDA", EINSTRUCTION::LDA, EAddressingMode::Immediate, 2, 2},
        {  "LDA", EINSTRUCTION::LDA, EAddressingMode::Absolute, 3, 4},
        {  "LDA", EINSTRUCTION::LDA, EAddressingMode::YIndirectIndexed, 2, 5},
        {  "LDA", EINSTRUCTION::LDA, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "LDA", EINSTRUCTION::LDA, EAddressingMode::YAbsoluteIndexed, 3, 4},
        {  "LDA", EINSTRUCTION::LDA, EAddressingMode::XAbsoluteIndexed, 3, 4},

        // Blue Block
        {  "LDX", EINSTRUCTION::LDX, EAddressingMode::Immediate, 2, 2},
        {  "LDX", EINSTRUCTION::LDX, EAddressingMode::Zeropage, 2, 3},
        {  "TAX", EINSTRUCTION::TAX, EAddressingMode::Implicit, 1, 2},
        {  "LDX", EINSTRUCTION::LDX, EAddressingMode::Absolute, 3, 4},
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "LDX", EINSTRUCTION::LDX, EAddressingMode::YZeroPageIndexed, 2, 4},
        {  "TSX", EINSTRUCTION::TSX, EAddressingMode::Implicit, 1, 2},
        {  "LDX", EINSTRUCTION::LDX, EAddressingMode::YAbsoluteIndexed, 3, 4},

        // Grey Block
        {  "LAX", EINSTRUCTION::LAX, EAddressingMode::XIndexedIndirect, 2, 255},
        {  "LAX", EINSTRUCTION::LAX, EAddressingMode::Zeropage, 2, 255},
        {  "LAX", EINSTRUCTION::LAX, EAddressingMode::Immediate, 2, 255},
        {  "LAX", EINSTRUCTION::LAX, EAddressingMode::Absolute, 3, 255},
        {  "LAX", EINSTRUCTION::LAX, EAddressingMode::YIndirectIndexed, 2, 255},
        {  "LAX", EINSTRUCTION::LAX, EAddressingMode::YZeroPageIndexed, 2, 255},
        {  "LAS", EINSTRUCTION::LAS, EAddressingMode::YAbsoluteIndexed, 3, 255},
        {  "LAX", EINSTRUCTION::LAX, EAddressingMode::YAbsoluteIndexed, 3, 255},
    }},
    // C0
    {{
        // Red Block
        {  "CPY", EINSTRUCTION::CPY, EAddressingMode::Immediate, 2, 2},
        {  "CPY", EINSTRUCTION::CPY, EAddressingMode::Zeropage, 2, 3},
        {  "INY", EINSTRUCTION::INY, EAddressingMode::Implicit, 1, 2},
        {  "CPY", EINSTRUCTION::CPY, EAddressingMode::Absolute, 3, 4},
        {  "BNE", EINSTRUCTION::BNE, EAddressingMode::Relative, 2, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XZeroPageIndexed, 1, 2},
        {  "CLD", EINSTRUCTION::CLD, EAddressingMode::Implicit, 1, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XZeroPageIndexed, 1, 2},

        // Green Block
        {  "CMP", EINSTRUCTION::CMP, EAddressingMode::XIndexedIndirect, 2, 6},
        {  "CMP", EINSTRUCTION::CMP, EAddressingMode::Zeropage,2, 3},
        {  "CMP", EINSTRUCTION::CMP, EAddressingMode::Immediate, 2, 2},
        {  "CMP", EINSTRUCTION::CMP, EAddressingMode::Absolute, 3, 4},
        {  "CMP", EINSTRUCTION::CMP, EAddressingMode::YIndirectIndexed, 2, 5},
        {  "CMP", EINSTRUCTION::CMP, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "CMP", EINSTRUCTION::CMP, EAddressingMode::YAbsoluteIndexed, 3, 4},
        {  "CMP", EINSTRUCTION::CMP, EAddressingMode::XAbsoluteIndexed, 3, 4},

        // Blue Block
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Immediate, 1, 2},
        {  "DEC", EINSTRUCTION::DEC, EAddressingMode::Zeropage, 2, 5},
        {  "DEX", EINSTRUCTION::DEX, EAddressingMode::Implicit, 1, 2},
        {  "DEC", EINSTRUCTION::DEC, EAddressingMode::Absolute, 3, 6},
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "DEC", EINSTRUCTION::DEC, EAddressingMode::XZeroPageIndexed, 2, 6},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Implicit, 1, 2},
        {  "DEC", EINSTRUCTION::DEC, EAddressingMode::XAbsoluteIndexed, 3, 7},

        // Grey Block
        {  "DCP", EINSTRUCTION::DCP, EAddressingMode::XIndexedIndirect, 2, 255},
        {  "DCP", EINSTRUCTION::DCP, EAddressingMode::Zeropage, 2, 255},
        {  "AXS", EINSTRUCTION::AXS, EAddressingMode::Immediate, 1, 255},
        {  "DCP", EINSTRUCTION::DCP, EAddressingMode::Absolute, 3, 255},
        {  "DCP", EINSTRUCTION::DCP, EAddressingMode::YIndirectIndexed, 2, 255},
        {  "DCP", EINSTRUCTION::DCP, EAddressingMode::XZeroPageIndexed, 2, 255},
        {  "DCP", EINSTRUCTION::DCP, EAddressingMode::YAbsoluteIndexed, 3, 255},
        {  "DCP", EINSTRUCTION::DCP, EAddressingMode::XAbsoluteIndexed, 3, 255}
    }},
    // E0
    {{
        // Red Block
        {  "CPX", EINSTRUCTION::CPX, EAddressingMode::Immediate, 2, 2},
        {  "CPX", EINSTRUCTION::CPX, EAddressingMode::Zeropage, 2, 3},
        {  "INX", EINSTRUCTION::INX, EAddressingMode::Implicit, 1, 2},
        {  "CPX", EINSTRUCTION::CPX, EAddressingMode::Absolute, 3, 4},
        {  "BEQ", EINSTRUCTION::BEQ, EAddressingMode::Relative, 2, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XZeroPageIndexed, 1, 2},
        {  "SED", EINSTRUCTION::SED, EAddressingMode::Implicit, 1, 2},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::XAbsoluteIndexed, 1, 2},

        // Green Block
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::XIndexedIndirect, 2, 6},
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::Zeropage, 2, 3},
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::Immediate, 2, 2},
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::Absolute, 3, 4},
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::YIndirectIndexed, 2, 5},
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::XZeroPageIndexed, 2, 4},
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::YAbsoluteIndexed, 3, 4},
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::XAbsoluteIndexed, 3, 4},

        // Blue Block
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Immediate, 1, 2},
        {  "INC", EINSTRUCTION::INC, EAddressingMode::Zeropage, 2, 5},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Implicit, 1, 2},
        {  "INC", EINSTRUCTION::INC, EAddressingMode::Absolute, 3, 6},
        {  "STP", EINSTRUCTION::STP, EAddressingMode::Implicit, 0, 255},
        {  "INC", EINSTRUCTION::INC, EAddressingMode::XZeroPageIndexed, 2, 6},
        {  "NOP", EINSTRUCTION::NOP, EAddressingMode::Implicit, 1, 2},
        {  "INC", EINSTRUCTION::INC, EAddressingMode::XAbsoluteIndexed, 3, 7},

        // Grey Block
        {  "ISC", EINSTRUCTION::ISC, EAddressingMode::XIndexedIndirect, 2, 255},
        {  "ISC", EINSTRUCTION::ISC, EAddressingMode::Zeropage, 2, 255},
        {  "SBC", EINSTRUCTION::SBC, EAddressingMode::Immediate, 2, 2},
        {  "ISC", EINSTRUCTION::ISC, EAddressingMode::Absolute, 3, 255},
        {  "ISC", EINSTRUCTION::ISC, EAddressingMode::YIndirectIndexed, 2, 255},
        {  "ISC", EINSTRUCTION::ISC, EAddressingMode::XZeroPageIndexed, 2, 255},
        {  "ISC", EINSTRUCTION::ISC, EAddressingMode::YAbsoluteIndexed, 3, 255},
        {  "ISC", EINSTRUCTION::ISC, EAddressingMode::XAbsoluteIndexed, 3, 255}
    }}
}};



NESOpCode OpCodeDecoder::DecodeOpCode(uint8_t AAA, uint8_t BBB, uint8_t CC)
{
    // Should be 0-7
    int32_t Row = AAA;
    int32_t Column = 0;

    // Implicit third push from 0b11 passing 0b01 and 0b10 tests
    Column += (CC & 0b01) != 0 ? 8 : 0;
    Column += (CC & 0b10) != 0 ? 16 : 0;

    NESOpCode Output = OpCodeTable[Row][Column + BBB];

    return Output;
}
