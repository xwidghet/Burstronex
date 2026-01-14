#include "APU.h"

#include "MemoryMapper.h"

#include <cstring>

void APU::Init(MemoryMapper* MemoryMapper)
{
    mRAM = MemoryMapper;
    mCyclesToRun = 0;

    std::memset(&mRegisters, 0, sizeof(mRegisters));
}

void APU::UpdateRegisters()
{
    mRegisters.Pulse1_Timer = mRAM->Read8Bit(PULSE1_TIMER_ADDRESS);
    mRegisters.Pulse1_LengthCounter = mRAM->Read8Bit(PULSE1_LENGTHCOUNTER_ADDRESS);
    mRegisters.Pulse1_Envelope = mRAM->Read8Bit(PULSE1_ENVELOPE_ADDRESS);
    mRegisters.Pulse1_Sweep = mRAM->Read8Bit(PULSE1_SWEEP_ADDRESS);

    mRegisters.Pulse2_Timer = mRAM->Read8Bit(PULSE2_TIMER_ADDRESS);
    mRegisters.Pulse2_LengthCounter = mRAM->Read8Bit(PULSE2_LENGTHCOUNTER_ADDRESS);
    mRegisters.Pulse2_Envelope = mRAM->Read8Bit(PULSE2_ENVELOPE_ADDRESS);
    mRegisters.Pulse2_Sweep = mRAM->Read8Bit(PULSE2_SWEEP_ADDRESS);

    mRegisters.Triangle_Timer = mRAM->Read8Bit(TRIANGLE_TIMER_ADDRESS);
    mRegisters.Triangle_LengthCounter = mRAM->Read8Bit(TRIANGLE_LENGTHCOUNTER_ADDRESS);
    mRegisters.Triangle_LinearCounter = mRAM->Read8Bit(TRIANGLE_LINEARCOUNTER_ADDRESS);

    mRegisters.Noise_Timer = mRAM->Read8Bit(NOISE_TIMER_ADDRESS);
    mRegisters.Noise_LengthCounter = mRAM->Read8Bit(NOISE_LENGTHCOUNTER_ADDRESS);
    mRegisters.Noise_Envelope = mRAM->Read8Bit(NOISE_ENVELOPE_ADDRESS);
    mRegisters.Noise_LinearFeedbackShiftRegister = mRAM->Read8Bit(NOISE_LINEARFEEDBACKSHIFTREGISTER_ADDRESS);

    mRegisters.DMC_Timer = mRAM->Read8Bit(DMC_TIMER_ADDRESS);
    mRegisters.DMC_MemoryReader = mRAM->Read8Bit(DMC_MEMORYREADER_ADDRESS);
    mRegisters.DMC_SampleBuffer = mRAM->Read8Bit(DMC_SAMPLEBUFFER_ADDRESS);
    mRegisters.DMC_OutputUnit = mRAM->Read8Bit(DMC_OUTPUTUNIT_ADDRESS);

    mRegisters.ChannelEnable_LengthCounterStatus = mRAM->Read8Bit(CHANNELENABLE_LENGTHCOUNTERSTATUS_ADDRESS);
    mRegisters.FrameCounter = mRAM->Read8Bit(FRAMECOUNTER_ADDRESS);
}

void APU::Execute(const uint8_t CPUCycles)
{
    UpdateRegisters();

    mCyclesToRun += CPUCycles;
    while(mCyclesToRun > 0)
    {
        ExecuteCycle();

        // APU operates at half the speed of the CPU.
        mCyclesToRun -= 2;
    }
}

void APU::ExecuteCycle()
{

}
