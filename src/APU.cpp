#include "APU.h"

#include "CPU.h"
#include "MemoryMapper.h"
#include "RomParameters.h"

#include <cstring>

void APU::Init(MemoryMapper* MemoryMapper, CPU* CPU, const ECPU_TIMING Timing)
{
    mRAM = MemoryMapper;
    mCPU = CPU;

    mCyclesToRun = 0;
    mCyclesSinceFrameInterrupt = 0;
    mSequenceCycleCount = 0;
    mSequenceIndex = 0;

    if (Timing == ECPU_TIMING::PAL)
    {
        mFrameInterruptCycleCount = PAL_FRAME_INTERRUPT_CYCLE_COUNT;
        mSequenceCycleInterval = PAL_SEQUENCE_STEP_CYCLE_COUNT;
    }
    else
    {
        mFrameInterruptCycleCount = NTSC_FRAME_INTERRUPT_CYCLE_COUNT;
        mSequenceCycleInterval = NTSC_SEQUENCE_STEP_CYCLE_COUNT;
    }

    std::memset(&mRegisters, 0, sizeof(mRegisters));
}

void APU::UpdateRegisters()
{
    mRegisters.Pulse1_Timer = mRAM->ReadRegister(PULSE1_TIMER_ADDRESS);
    mRegisters.Pulse1_LengthCounter = mRAM->ReadRegister(PULSE1_LENGTHCOUNTER_ADDRESS);
    mRegisters.Pulse1_Envelope = mRAM->ReadRegister(PULSE1_ENVELOPE_ADDRESS);
    mRegisters.Pulse1_Sweep = mRAM->ReadRegister(PULSE1_SWEEP_ADDRESS);

    mRegisters.Pulse2_Timer = mRAM->ReadRegister(PULSE2_TIMER_ADDRESS);
    mRegisters.Pulse2_LengthCounter = mRAM->ReadRegister(PULSE2_LENGTHCOUNTER_ADDRESS);
    mRegisters.Pulse2_Envelope = mRAM->ReadRegister(PULSE2_ENVELOPE_ADDRESS);
    mRegisters.Pulse2_Sweep = mRAM->ReadRegister(PULSE2_SWEEP_ADDRESS);

    mRegisters.Triangle_Timer = mRAM->ReadRegister(TRIANGLE_TIMER_ADDRESS);
    mRegisters.Triangle_LengthCounter = mRAM->ReadRegister(TRIANGLE_LENGTHCOUNTER_ADDRESS);
    mRegisters.Triangle_LinearCounter = mRAM->ReadRegister(TRIANGLE_LINEARCOUNTER_ADDRESS);

    mRegisters.Noise_Timer = mRAM->ReadRegister(NOISE_TIMER_ADDRESS);
    mRegisters.Noise_LengthCounter = mRAM->ReadRegister(NOISE_LENGTHCOUNTER_ADDRESS);
    mRegisters.Noise_Envelope = mRAM->ReadRegister(NOISE_ENVELOPE_ADDRESS);
    mRegisters.Noise_LinearFeedbackShiftRegister = mRAM->ReadRegister(NOISE_LINEARFEEDBACKSHIFTREGISTER_ADDRESS);

    mRegisters.DMC_Timer = mRAM->ReadRegister(DMC_TIMER_ADDRESS);
    mRegisters.DMC_MemoryReader = mRAM->ReadRegister(DMC_MEMORYREADER_ADDRESS);
    mRegisters.DMC_SampleBuffer = mRAM->ReadRegister(DMC_SAMPLEBUFFER_ADDRESS);
    mRegisters.DMC_OutputUnit = mRAM->ReadRegister(DMC_OUTPUTUNIT_ADDRESS);

    mRegisters.Status = mRAM->ReadRegister(STATUS_ADDRESS);
    mRegisters.FrameCounter = mRAM->ReadRegister(FRAMECOUNTER_ADDRESS);
}

void APU::Execute(const uint8_t CPUCycles)
{
    UpdateRegisters();

    mCyclesToRun += CPUCycles;
    mCyclesSinceFrameInterrupt += mCyclesSinceFrameInterrupt;
    while(mCyclesToRun > 0)
    {
        ExecuteCycle();
        ExecuteSequencer();

        // APU operates at half the speed of the CPU.
        mCyclesToRun -= 2;
    }
}

void APU::ExecuteCycle()
{

}

void APU::ExecuteSequencer()
{
    bool Mode = (mRegisters.FrameCounter & static_cast<uint8_t>(EFRAME_COUNTER_MASKS::MODE)) != 0;

    if (mCyclesSinceFrameInterrupt >= mFrameInterruptCycleCount)
    {
        mCyclesSinceFrameInterrupt = 0;

        bool bBlockIRQ = (mRegisters.FrameCounter & static_cast<uint8_t>(EFRAME_COUNTER_MASKS::IRQ_INHIBIT)) != 0;

        if (Mode == 0 && !bBlockIRQ)
        {
            mRegisters.Status |= static_cast<uint8_t>(ESTATUS_READ_MASKS::FRAME_INTERRUPT);
            mRAM->WriteRegister(STATUS_ADDRESS, mRegisters.Status);
            mCPU->SetIRQ(true);
        }
    }

    if ((mSequenceCycleCount % mSequenceCycleInterval) == 0)
    {
        if (Mode == 0)
            ExecuteMode0Sequencer();
        else
            ExecuteMode1Sequencer();
    }
}

void APU::ExecuteMode0Sequencer()
{
    switch (mSequenceIndex)
    {
        case 0:
        case 1:
        case 2:
        case 3:
            break;
    }

    mSequenceIndex = (mSequenceIndex + 1) % 4;
}

void APU::ExecuteMode1Sequencer()
{
    switch (mSequenceIndex)
    {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
            break;
    }

    mSequenceIndex = (mSequenceIndex + 1) % 5;
}
