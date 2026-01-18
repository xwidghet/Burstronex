#include "APU.h"

#include "CPU.h"
#include "CircularBuffer.h"
#include "Logger.h"
#include "MemoryMapper.h"
#include "RomParameters.h"

#include "miniaudio.c"

#include <algorithm>
#include <cstring>

constexpr std::array<float, 31> GenerateSquareTable()
{
    std::array<float, 31> SquareTable = {};
    for (int i = 1; i <= 31; i++)
    {
        SquareTable[i-1] = 95.52 / (8128.0 / double(i) + 100);
    }

    return SquareTable;
}

constexpr std::array<float, 203>  GenerateTND()
{
    std::array<float, 203> TNDTable = {};
    for (int i = 1; i <= 203; i++)
    {
        TNDTable[i-1] = 163.67 / (24329.0 / double(i) + 100);
    }

    return TNDTable;
}

// Pulse1 and Pulse2 square wave DAC Output Table.
static constexpr std::array<float, 31> SquareLUT = GenerateSquareTable();

// Triangle, Noise, and DMC DAC Output Table
static constexpr std::array<float, 203> TNDLUT =  GenerateTND();

void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount)
{
    // In playback mode copy data to pOutput. In capture mode read data from pInput. In full-duplex mode, both
    // pOutput and pInput will be valid and you can move data from pInput into pOutput. Never process more than
    // frameCount frames.
    auto& Buffer = *reinterpret_cast<CircularBuffer<float, NTSC_FRAME_INTERRUPT_CYCLE_COUNT*3>*>(pDevice->pUserData);
    size_t ReadIndex = 0;
    size_t EndIndex = 0;

    Buffer.GetBucket(frameCount, ReadIndex, EndIndex);
    frameCount -= (EndIndex - ReadIndex);

    float* OutputBuffer = (float*)pOutput;
    while(ReadIndex <= EndIndex)
    {
        *OutputBuffer = Buffer[ReadIndex++];
        OutputBuffer++;
    }

    // First bucket may have reached the end of the array, requiring a second pass.
    if (frameCount > 0)
    {
        Buffer.GetBucket(frameCount, ReadIndex, EndIndex);

        while(ReadIndex <= EndIndex)
        {
            *OutputBuffer = Buffer[ReadIndex++];
            OutputBuffer++;
        }
    }
}

APU::APU()
{
    mAudioDevice = std::make_unique<ma_device>();
}

APU::~APU()
{
    ma_device_uninit(&*mAudioDevice);
}

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

    mPulse2.mbIsPulse2 = true;

    mbStartedDevice = false;

    std::memset(&mRegisters, 0, sizeof(mRegisters));

    InitAudio();
}

void APU::InitAudio()
{
    ma_device_config config = ma_device_config_init(ma_device_type_playback);

    // Make miniaudio do the legwork of converting the APU's 1.79M sample rate to 48khz.
    config.playback.format   = ma_format_f32;   // Set to ma_format_unknown to use the device's native format.
    config.playback.channels = 1;               // Set to 0 to use the device's native channel count.
    config.sampleRate        = std::ceil(mCPU->GetClockFrequency());           // Set to 0 to use the device's native sample rate.
    config.dataCallback      = data_callback;   // This function will be called when miniaudio needs more data.
    config.pUserData         = &mAudioBuffer;   // Can be accessed from the device object (device.pUserData).

    // APU output should always be [0-1], converted to [-1,1], so disable clipping for performance improvement.
    config.noClip = true;

    // On Linux with Pipewire thorugh alsa backend, it seems shared mode only partially works:
    // If GNES starts first: Behavior is of exclusive mode, other applications can't play audio.
    // If GNES starts second: Other applications behave normally, but GNES only outputs corrupted audio with cuts.
    config.playback.shareMode = ma_share_mode_shared;
    config.resampling.sampleRateIn = config.sampleRate;
    config.resampling.sampleRateOut = TARGET_SAMPLE_RATE;
    config.resampling.channels = 2;

    mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::ERROR, "Config {0}\n", config.periodSizeInFrames);

    ma_result result = ma_device_init(NULL, &config, &*mAudioDevice);
    if (result != MA_SUCCESS) {
        mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::ERROR, "Failed to initialize miniaudio device: {0}\n", static_cast<uint8_t>(result));
    }
}

void APU::UpdateRegisters()
{
    mRegisters.Pulse1_Timer = mRAM->ReadRegister(PULSE1_TIMER_ADDRESS);
    mRegisters.Pulse1_LengthCounter = mRAM->ReadRegister(PULSE1_LENGTHCOUNTER_ADDRESS);
    mRegisters.Pulse1_Envelope = mRAM->ReadRegister(PULSE1_ENVELOPE_ADDRESS);

    uint8_t NewSweepData = mRAM->ReadRegister(PULSE1_SWEEP_ADDRESS);
    if (NewSweepData != mRegisters.Pulse1_Sweep)
    {
        mRegisters.Pulse1_Sweep = mRAM->ReadRegister(PULSE1_SWEEP_ADDRESS);
        uint8_t LengthCounterIndex = (mRegisters.Pulse1_LengthCounter & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
        mPulse1.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];

        bool bConstantVolume = (mRegisters.Pulse1_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::CONSTANT_VOLUME)) != 0;
        if (bConstantVolume)
        {
            mPulse1.mEnvelope.Value = (mRegisters.Pulse1_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::VOLUME_ENVELOPE)) & 0b1111;
        }
        else
        {
            mPulse1.mEnvelope.Value = 15;
        }
    }
    

    mRegisters.Pulse2_Timer = mRAM->ReadRegister(PULSE2_TIMER_ADDRESS);
    mRegisters.Pulse2_LengthCounter = mRAM->ReadRegister(PULSE2_LENGTHCOUNTER_ADDRESS);
    mRegisters.Pulse2_Envelope = mRAM->ReadRegister(PULSE2_ENVELOPE_ADDRESS);

    NewSweepData = mRAM->ReadRegister(PULSE2_SWEEP_ADDRESS);
    if (NewSweepData != mRegisters.Pulse2_Sweep)
    {
        mRegisters.Pulse2_Sweep = mRAM->ReadRegister(PULSE2_SWEEP_ADDRESS);
        uint8_t LengthCounterIndex = (mRegisters.Pulse2_LengthCounter & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
        mPulse2.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];

        bool bConstantVolume = (mRegisters.Pulse2_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::CONSTANT_VOLUME)) != 0;
        if (bConstantVolume)
        {
            mPulse1.mEnvelope.Value = (mRegisters.Pulse2_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::VOLUME_ENVELOPE)) & 0b1111;
        }
        else
        {
            mPulse1.mEnvelope.Value = 15;
        }
    }

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

    auto NewStatus = mRAM->ReadRegister(STATUS_ADDRESS);
    if (mRegisters.Status != NewStatus)
    {
        mRegisters.Status = NewStatus;

        bool bPulse1LCHalt = !(mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_1_PLAYING));
        bool bPulse2LCHalt = !(mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_2_PLAYING));

        mPulse1.mLengthCounter = bPulse1LCHalt ? 0 : mPulse1.mLengthCounter;
        mPulse2.mLengthCounter = bPulse2LCHalt ? 0 : mPulse2.mLengthCounter;
    }
     
    mRegisters.FrameCounter = mRAM->ReadRegister(FRAMECOUNTER_ADDRESS);
}

void APU::Execute(const uint8_t CPUCycles)
{
    UpdateRegisters();

    mCyclesToRun += CPUCycles;
    mCyclesSinceFrameInterrupt += CPUCycles;
    while(mCyclesToRun > 0)
    {
        ExecuteCycle();
        ExecuteSequencer();

        // APU operates at half the speed of the CPU.
        mCyclesToRun -= 2;
    }

    // Every CPU Cycle the APU outputs one sample
    for (int i = 0; i < CPUCycles; i++)
    {
        //float NewSample = TestOutput(CPUCycles, i);
        float NewSample = DACOutput()*2.f - 1.f;

        mAudioBuffer.Push(NewSample);
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
        mAudioFrameCount++;

        bool bBlockIRQ = (mRegisters.FrameCounter & static_cast<uint8_t>(EFRAME_COUNTER_MASKS::IRQ_INHIBIT)) != 0;

        if (Mode == 0 && !bBlockIRQ)
        {
            mRegisters.Status |= static_cast<uint8_t>(ESTATUS_READ_MASKS::FRAME_INTERRUPT);
            mRAM->WriteRegister(STATUS_ADDRESS, mRegisters.Status);
            mCPU->SetIRQ(true);
        }

        // Start audio device once we have one audio frame of data generated.
        if (!mbStartedDevice)
        {
            mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::INFO, "Started Audio Device\n");
            ma_device_start(&*mAudioDevice);
            mbStartedDevice = true;
        }

        mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::INFO, "APU Buffer: {0}%\n", mAudioBuffer.GetPercentageFilled());
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
        QuarterFrame();
        break;
    case 1:
        HalfFrame();
        QuarterFrame();
        break;
    case 2:
        QuarterFrame();
        break;
    case 3:
        HalfFrame();
        QuarterFrame();
        break;
    }

    mSequenceIndex = (mSequenceIndex + 1) % 4;
}

void APU::ExecuteMode1Sequencer()
{
    switch (mSequenceIndex)
    {
        case 0:
            QuarterFrame();
            break;
        case 1:
            HalfFrame();
            QuarterFrame();
            break;
        case 2:
            QuarterFrame();
            break;
        case 3:
            // Nothing Happens (TM?)
            break;
        case 4:
            HalfFrame();
            QuarterFrame();
            break;
    }

    mSequenceIndex = (mSequenceIndex + 1) % 5;
}

void APU::HalfFrame()
{
    bool bPulse1Infinite = (mRegisters.Pulse1_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::ENVELOPE_LOOP));
    bool bPulse2Infinite = (mRegisters.Pulse2_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::ENVELOPE_LOOP));
    mPulse1.ClockLengthCounter(bPulse1Infinite);
    mPulse2.ClockLengthCounter(bPulse2Infinite);

    mPulse1.ClockSweep(mRegisters.Pulse1_LengthCounter, mRegisters.Pulse1_Envelope, mRegisters.Pulse1_Sweep);
    mPulse2.ClockSweep(mRegisters.Pulse2_LengthCounter, mRegisters.Pulse2_Envelope, mRegisters.Pulse2_Sweep);

    // Clock Length Counters
}

void APU::QuarterFrame()
{
    // Clock Envelopes and Triangle linear counter.
    mPulse1.ClockEnvelope(mRegisters.Pulse1_Timer);
    mPulse2.ClockEnvelope(mRegisters.Pulse2_Timer);
}

float APU::DACOutput()
{
    uint8_t Triangle = 0;
    uint8_t Noise = 0;
    uint8_t DMC = 0;

    uint8_t Pulse1 = mPulse1.Execute(mRegisters.Pulse1_Timer, mRegisters.Pulse1_LengthCounter, mRegisters.Pulse1_Envelope, mRegisters.Pulse1_Sweep);
    uint8_t Pulse2 = mPulse2.Execute(mRegisters.Pulse2_Timer, mRegisters.Pulse2_LengthCounter, mRegisters.Pulse2_Envelope, mRegisters.Pulse2_Sweep);

    mRAM->WriteRegister(PULSE1_TIMER_ADDRESS, mRegisters.Pulse1_Timer);
    mRAM->WriteRegister(PULSE1_LENGTHCOUNTER_ADDRESS, mRegisters.Pulse1_LengthCounter);
    mRAM->WriteRegister(PULSE1_ENVELOPE_ADDRESS, mRegisters.Pulse1_Envelope);
    mRAM->WriteRegister(PULSE1_SWEEP_ADDRESS, mRegisters.Pulse1_Sweep);

    mRAM->WriteRegister(PULSE2_TIMER_ADDRESS, mRegisters.Pulse2_Timer);
    mRAM->WriteRegister(PULSE2_LENGTHCOUNTER_ADDRESS, mRegisters.Pulse2_LengthCounter);
    mRAM->WriteRegister(PULSE2_ENVELOPE_ADDRESS, mRegisters.Pulse2_Envelope);
    mRAM->WriteRegister(PULSE2_SWEEP_ADDRESS, mRegisters.Pulse2_Sweep);

    // Pulse, Triangle, and Noise should be 0-15
    // DMC should be 0-127
    // DAC Output is a float in the range [0.0, 1.0]
    float PulseOut = SquareLUT[Pulse1 + Pulse2];

    float TNDOut = TNDLUT[3*Triangle + 2*Noise + DMC];

    return PulseOut + TNDOut;
}

float APU::GetBufferFillPercentage() const
{
    if (mbStartedDevice)
        return mAudioBuffer.GetPercentageFilled();
    else
        return 1.f;
}

float APU::TestOutput(uint8_t CPUCycles, uint8_t i)
{
    float Radian = std::fmod(double(mCPU->GetCycleCount() - CPUCycles + i), mCPU->GetClockFrequency());
    Radian /= mCPU->GetClockFrequency();

    /*
     float Pulse1 = (sin(Radian* * 6.283185307f) + 1) * 7;

     float NextSample = DACOutput(Pulse1, Pulse1, Pulse1, Pulse1, Pulse1*2);
     NextSample = NextSample*2.f - 1.f;
     */

    return sin(Radian * 6.283185307f * (200 + 25 * cos(6.283185307f * Radian)));
}

uint8_t APU::PulseUnit::Execute(uint8_t& TimerRegister, uint8_t& LengthCounterRegister, uint8_t& EnvelopeRegister, uint8_t& SweepRegister)
{
    uint8_t Value = mEnvelope.Value;
    Value *= !mSweep.mbIsMutingChannel;
    Value *= mLengthCounter > 0;

    uint16_t Timer = EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::TIMER_LOW);
    Timer |= (LengthCounterRegister & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;

    // Sequencer Begin
    Timer -= 1;

    uint8_t Duty = (TimerRegister & static_cast<uint8_t>(EPULSE_TIMER_MASKS::DUTY)) >> 6;
    uint8_t SequencerOutput = (DUTY_CYCLE_SEQUENCES[Duty] & (1 << (Timer % 8))) != 0;
    Value *= SequencerOutput;

    EnvelopeRegister &= ~static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::TIMER_LOW);
    EnvelopeRegister |= Timer & 0xFF;

    LengthCounterRegister &= ~static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH);
    LengthCounterRegister |= (Timer >> 8);

    Value *= Timer < 8 ? 0 : 1;
    // Sequencer End

    return Value;
}

void APU::PulseUnit::ClockEnvelope(uint8_t& TimerRegister)
{
    bool bConstantVolume = (TimerRegister & static_cast<uint8_t>(EPULSE_TIMER_MASKS::CONSTANT_VOLUME)) != 0;
    if (!bConstantVolume)
    {
        if (mEnvelope.Value == 0)
        {
            bool bLoop = (TimerRegister & static_cast<uint8_t>(EPULSE_TIMER_MASKS::ENVELOPE_LOOP)) != 0;
            mEnvelope.Value = bLoop ? 15 : 0;
        }
        else
        {
            uint8_t Rate = (TimerRegister & static_cast<uint8_t>(EPULSE_TIMER_MASKS::VOLUME_ENVELOPE)) & 0b1111;
            mEnvelope.Value = mEnvelope.Value >= Rate ? mEnvelope.Value - Rate : 0;
        }
    }
}

void APU::PulseUnit::ClockSweep(uint8_t& LengthCounterRegister, uint8_t& EnvelopeRegister, uint8_t& SweepRegister)
{
    uint16_t Timer = EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::TIMER_LOW);
    Timer |= (LengthCounterRegister & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;

    mSweep.mbIsEnabled = SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_ENABLED);
    uint8_t ShiftCount = SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_SHIFT);

    // Pulse Square Wave Period ??
    int32_t Period = (SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_PERIOD)) >> 4;

    if (mLastPeriod != Period || mbLastSweepEnabled != mSweep.mbIsEnabled)
    {
        mTargetPeriod = mCurrentPeriod;
    }

    if (mSweep.mbIsEnabled && (ShiftCount > 0))
    {
        int32_t Change = ShiftCount == 0 ? mCurrentPeriod : 0;
        for (int i = 0; i < ShiftCount; i++)
        {
            Change |= Timer & (1 << i);
        }
        Timer = Timer >> ShiftCount;

        bool bNegateFlag = (SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_NEGATE)) != 0;

        // Pulse1 uses ones compliment
        Change = bNegateFlag ? (-Change + !mbIsPulse2) : Change;

        mTargetPeriod = Period + std::max(Period + Change, 0);

        mSweep.mDivider -= 1;
    }

    mSweep.mbIsMutingChannel = mTargetPeriod > 0x7FF ? 0 : 1;
    mSweep.mbIsMutingChannel |= Period < 8 ? 0 : 1;
}

void APU::PulseUnit::ClockLengthCounter(bool bInfinite)
{
    // Infinite doesn't modify it at all, so if it's at 0 it stays at zero.
    if (bInfinite)
        return;

    mLengthCounter = mLengthCounter > 0 ? mLengthCounter-- : 0;
}