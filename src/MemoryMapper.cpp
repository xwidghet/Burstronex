#include "MemoryMapper.h"

#include "APU.h"
#include "CPU.h"
#include "PPU.h"
#include "Input.h"
#include "Renderer.h"
#include "OpCodeDecoder.h"

#include "Logger.h"

#include <cstring>

MemoryMapper::MemoryMapper(const std::vector<char>& ChrRomMemory, const std::vector<char>& PrgRomMemory)
{
    // Supposedly I should make this a 2D array...I'll do that later when I learn why
    mMemory = std::vector<uint8_t>(65536);

    mChrRomMemory = ChrRomMemory;
    mPrgRomMemory = PrgRomMemory;

    // What to do with trainer memory, do we need to increase size to load it?

    mPrgRomLocation =  0x8000;
    std::memcpy(mMemory.data() + mPrgRomLocation, PrgRomMemory.data(), PrgRomMemory.size());

    // Chr Memory requires a mapper to dynamically load information into 0x0000 -> 0x1FFF range during rendering.
    // Since it's only used for rendering, I can skip it for now.
    mChrRomLocation = 0x0000;

    mController1Shift = 0;
    mController2Shift = 0;
}

void MemoryMapper::Init(CPU* CPU, APU* APU, PPU* PPU, Renderer* RendererPtr)
{
    mCPU = CPU;
    mAPU = APU;
    mPPU = PPU;
    mRenderer = RendererPtr;
}

uint32_t MemoryMapper::MapAddress(uint32_t Address)
{
    // Internal RAM, 0x0000 - 0x07FF, Mirrored up to 0x1FFF
    if (Address <= 0x1FFF)
        Address = 0x0000 + (Address - 0x0000) % (0x07FF + 1);
    // NES PPU Registers, 0x2000 - 0x2007, Mirrored up to 0x3FFF
    else if (Address <= 0x3FFF)
        Address = 0x2000 + (Address - 0x2000) % ((0x2007 - 0x2000) + 1);
    // Prg ROM Area, with iNES 2.0 ROMS it could be as small as 8KB X 4 Mirrors, but generally 16KB X 2 Mirrors or 32KB No Mirrors
    else if (Address >= 0x8000)
        Address = 0x8000 + (Address - 0x8000) % (mPrgRomMemory.size());

    return Address;
}

uint8_t MemoryMapper::Read8Bit(const uint32_t Address)
{
    uint32_t TargetAddress = MapAddress(Address);
    uint8_t Value = mMemory[TargetAddress];

    if (TargetAddress == STATUS_ADDRESS)
    {
        return mAPU->ReadStatus();
    }
    else if (TargetAddress >= PPUCTRL_ADDRESS && TargetAddress <= PPUDATA_ADDRESS)
    {
        switch (TargetAddress)
        {
            case PPUSTATUS_ADDRESS:
                Value = mPPU->ReadPPUSTATUS();
                break;
            case PPUDATA_ADDRESS:
                Value = mPPU->ReadPPUData();
                break;
            case OAMDATA_ADDRESS:
                Value = mPPU->ReadOAMDATA();
                break;
            default:
                Value = 0;
        }
    }
    else if (TargetAddress == CONTROLLER_1_READ_ADDRESS)
    {
        // Official controllers always return 1 once the inputs have been exhausted
        if (mController1Shift > 7)
        {
            mController1Shift = 8;
            return 1;
        }
        Value = (mRenderer->GetController1() & (1 << mController1Shift)) != 0;
        mController1Shift++;

        mLog->Log(ELOGGING_SOURCES::MEMORY_MAPPER, ELOGGING_MODE::INFO, "Controller 1 Button: {0} {1}\n", mController1Shift-1, Value);
    }
    else if (TargetAddress == CONTROLLER_2_READ_ADDRESS)
    {
        if (mController2Shift > 7)
        {
            mController2Shift = 8;
            return 1;
        }
        Value = (mRenderer->GetController2() & (1 << mController2Shift)) != 0;
        mController2Shift++;
    }

    return Value;
}

void MemoryMapper::Write8Bit(const uint32_t Address, uint8_t Value)
{
    // Cannot write to Program Read Only Memory (ROM)
    if (Address >= mPrgRomLocation)
        return;

    uint32_t TargetAddress = MapAddress(Address);

    // OAMDMA not being in the PPU address range makes me sad :(
    // In the optimization phase I'll want to implement real memory mapping somehow to replace all of this.
    if (TargetAddress == OAMDMA_ADDRESS || (TargetAddress >= PPUCTRL_ADDRESS && TargetAddress <= PPUDATA_ADDRESS))
    {
        switch (TargetAddress)
        {
            case OAMADDR_ADDRESS:
                mPPU->WriteOAMADDR(Value);
                break;
            case OAMDATA_ADDRESS:
                mPPU->WriteOAMDATA(Value);
                break;
            case OAMDMA_ADDRESS:
                mPPU->WriteOAMDMA(Value);
                break;
            case PPUADDR_ADDRESS:
                mPPU->WritePPUADDR(Value);
                break;
            case PPUDATA_ADDRESS:
                mPPU->WritePPUData(Value);
                break;
            case PPUSCROLL_ADDRESS:
                mPPU->WritePPUSCROLL(Value);
                break;
            case PPUCTRL_ADDRESS:
                mPPU->WritePPUCTRL(Value);
                break;
            case PPUMASK_ADDRESS:
                mPPU->WritePPUMASK(Value);
                break;
            case PPUSTATUS_ADDRESS:
                // PPUSTATUS is the one register the CPU can't write.
                // However supposedly it affects the Open Bus values, so layer I'll need to implement something here.
                mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "REJECTED CPU WRITTING PPU STATUS!");
                break;
        }

        return;
    }
    else if (TargetAddress == CONTROLLER_STROBE_ADDRESS)
    {
        // First input always gets returned while Strobe bit is enabled.
        bool bStrobeHigh = (Value & static_cast<uint8_t>(EControllerReadMasks::PRIMARY_CONTROLLER_STATUS)) != 0;
        if (bStrobeHigh)
        {
            mController1Shift = 0;
            mController2Shift = 0;
        }
    }
    else if (TargetAddress >= PULSE1_ENVELOPE_ADDRESS && TargetAddress <= FRAMECOUNTER_ADDRESS)
    {
        switch (TargetAddress)
        {
            case PULSE1_TIMER_ADDRESS:
                mAPU->WritePulse1_Timer(Value);
                break;
            case PULSE1_LENGTHCOUNTER_ADDRESS:
                mAPU->WritePulse1_LengthCounter(Value);
                break;
            case PULSE1_ENVELOPE_ADDRESS:
                mAPU->WritePulse1_Envelope(Value);
                break;
            case PULSE1_SWEEP_ADDRESS:
                mAPU->WritePulse1_Sweep(Value);
                break;
            case PULSE2_TIMER_ADDRESS:
                mAPU->WritePulse2_Timer(Value);
                break;
            case PULSE2_LENGTHCOUNTER_ADDRESS:
                mAPU->WritePulse2_LengthCounter(Value);
                break;
            case PULSE2_ENVELOPE_ADDRESS:
                mAPU->WritePulse2_Envelope(Value);
                break;
            case PULSE2_SWEEP_ADDRESS:
                mAPU->WritePulse2_Sweep(Value);
                break;
            case TRIANGLE_TIMER_ADDRESS:
                mAPU->WriteTriangle_Timer(Value);
                break;
            case TRIANGLE_LENGTHCOUNTER_ADDRESS:
                mAPU->WriteTriangle_LengthCounter(Value);
                break;
            case TRIANGLE_LINEARCOUNTER_ADDRESS:
                mAPU->WriteTriangle_LinearCounter(Value);
                break;
            case NOISE_ENVELOPE_ADDRESS:
                mAPU->WriteNoise_Envelope(Value);
                break;
            case NOISE_UNUSED:
                break;
            case NOISE_MODE_PERIOD_ADDRESS:
                mAPU->WriteNoise_ModePeriod(Value);
                break;
            case NOISE_LENGTHCOUNTER_ADDRESS:
                mAPU->WriteNoise_LengthCounter(Value);
                break;
            case DMC_TIMER_ADDRESS:
                mAPU->WriteDMC_Timer(Value);
                break;
            case DMC_MEMORYREADER_ADDRESS:
                mAPU->WriteDMC_MemoryReader(Value);
                break;
            case DMC_SAMPLEBUFFER_ADDRESS:
                mAPU->WriteDMC_SampleBuffer(Value);
                break;
            case DMC_OUTPUTUNIT_ADDRESS:
                mAPU->WriteDMC_OutputUnit(Value);
                break;
            case STATUS_ADDRESS:
                mAPU->WriteStatus(Value);
                break;
            case FRAMECOUNTER_ADDRESS:
                mAPU->WriteFrameCounter(Value);
                break;
        }
        return;

    }

    mMemory[TargetAddress] = Value;
}

uint16_t MemoryMapper::Read16Bit(const uint32_t Address)
{
    uint16_t Low = mMemory[MapAddress(Address)];
    uint16_t High = mMemory[MapAddress(Address+1)];

    return uint16_t((High << 8) | Low);
}

void MemoryMapper::Write16Bit(const uint32_t Address, uint16_t Value)
{
    // Cannot write to Program Read Only Memory (ROM)
    if (Address >= mPrgRomLocation)
        return;

    uint16_t Low = Value & 0xFF;
    uint16_t High = (Value >> 8) & 0xFF;

    // What happens when it overflows?
    mMemory[MapAddress(Address)] = Low;
    mMemory[MapAddress(Address+1)] = High;
}

uint8_t MemoryMapper::ReadRegister(const uint32_t Address)
{
    return mMemory[MapAddress(Address)];
}

void MemoryMapper::WriteRegister(const uint32_t Address, uint8_t Value)
{
    // Cannot write to Program Read Only Memory (ROM)
    if (Address >= mPrgRomLocation)
        return;

    mMemory[MapAddress(Address)] = Value;
}

uint8_t* MemoryMapper::GetMemoryPtr(uint16_t TargetAddress)
{
    uint16_t Address = MapAddress(TargetAddress);

    return &mMemory[Address];
}

const uint16_t MemoryMapper::GetPrgRomLocation() const
{
    return mPrgRomLocation;
}

const uint16_t MemoryMapper::GetChrRomLocation() const
{
    return mChrRomLocation;
}
