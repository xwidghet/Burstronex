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
}

APU::~APU()
{
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

    mAudioDevice = std::make_unique<ma_device>();
    ma_result result = ma_device_init(NULL, &config, &*mAudioDevice);
    if (result != MA_SUCCESS) {
        mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::ERROR, "Failed to initialize miniaudio device: {0}\n", static_cast<uint8_t>(result));
    }
}

void APU::Execute(const uint8_t CPUCycles)
{
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

    mPulse1.ClockSweep(mRAM, mRegisters.Pulse1_LengthCounter, mRegisters.Pulse1_Envelope, mRegisters.Pulse1_Sweep);
    mPulse2.ClockSweep(mRAM, mRegisters.Pulse2_LengthCounter, mRegisters.Pulse2_Envelope, mRegisters.Pulse2_Sweep);

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

void APU::WritePulse1_Timer(const uint8_t Data)
{
    mRegisters.Pulse1_Timer = Data;
}

void APU::WritePulse1_LengthCounter(const uint8_t Data)
{
    mRegisters.Pulse1_LengthCounter = Data;
}

void APU::WritePulse1_Envelope(const uint8_t Data)
{
    mRegisters.Pulse1_Envelope = Data;
}

void APU::WritePulse1_Sweep(const uint8_t Data)
{
    mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::INFO, "Sweep data Updated!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

    mRegisters.Pulse1_Sweep = Data;

    uint8_t LengthCounterIndex = (mRegisters.Pulse1_LengthCounter & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
    mPulse1.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];

    mPulse1.mSequencerIndex = 0;

    bool bConstantVolume = (mRegisters.Pulse1_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::CONSTANT_VOLUME)) != 0;
    if (bConstantVolume)
    {
        mPulse1.mEnvelope.mValue = (mRegisters.Pulse1_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::VOLUME_ENVELOPE)) & 0b1111;
    }
    else
    {
        mPulse1.mEnvelope.mValue = 15;
    }
}

void APU::WritePulse2_Timer(const uint8_t Data)
{
    mRegisters.Pulse2_Timer = Data;
}

void APU::WritePulse2_LengthCounter(const uint8_t Data)
{
    mRegisters.Pulse2_LengthCounter = Data;
}

void APU::WritePulse2_Envelope(const uint8_t Data)
{
    mRegisters.Pulse2_Envelope = Data;
}

void APU::WritePulse2_Sweep(const uint8_t Data)
{
    mRegisters.Pulse2_Sweep = Data;
    uint8_t LengthCounterIndex = (mRegisters.Pulse2_LengthCounter & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
    mPulse2.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];

    mPulse2.mSequencerIndex = 0;

    bool bConstantVolume = (mRegisters.Pulse2_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::CONSTANT_VOLUME)) != 0;
    if (bConstantVolume)
    {
        mPulse2.mEnvelope.mValue = (mRegisters.Pulse2_Timer & static_cast<uint8_t>(EPULSE_TIMER_MASKS::VOLUME_ENVELOPE)) & 0b1111;
    }
    else
    {
        mPulse2.mEnvelope.mValue = 15;
    }
}

void APU::WriteTriangle_Timer(const uint8_t Data)
{
    mRegisters.Triangle_Timer = Data;
}

void APU::WriteTriangle_LengthCounter(const uint8_t Data)
{
    mRegisters.Triangle_LengthCounter = Data;
}

void APU::WriteTriangle_LinearCounter(const uint8_t Data)
{
    mRegisters.Triangle_LinearCounter = Data;
}

void APU::WriteNoise_Timer(const uint8_t Data)
{
    mRegisters.Noise_Timer = Data;
}

void APU::WriteNoise_LengthCounter(const uint8_t Data)
{
    mRegisters.Noise_LengthCounter = Data;
}

void APU::WriteNoise_Envelope(const uint8_t Data)
{
    mRegisters.Noise_Envelope = Data;
}

void APU::WriteNoise_LinearFeedbackShiftRegister(const uint8_t Data)
{
    mRegisters.Noise_LinearFeedbackShiftRegister = Data;
}

void APU::WriteDMC_Timer(const uint8_t Data)
{
    mRegisters.DMC_Timer = Data;
}

void APU::WriteDMC_MemoryReader(const uint8_t Data)
{
    mRegisters.DMC_MemoryReader = Data;
}

void APU::WriteDMC_SampleBuffer(const uint8_t Data)
{
    mRegisters.DMC_SampleBuffer = Data;
}

void APU::WriteDMC_OutputUnit(const uint8_t Data)
{
    mRegisters.DMC_OutputUnit = Data;
}

uint8_t APU::ReadStatus()
{
    return mRegisters.Status;
}

void APU::WriteStatus(const uint8_t Data)
{
    mRegisters.Status = Data;

    // Todo: Implement all the other channels
    bool bPulse1LCHalt = !(mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_1_PLAYING));
    bool bPulse2LCHalt = !(mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_2_PLAYING));

    mPulse1.mLengthCounter = bPulse1LCHalt ? 0 : mPulse1.mLengthCounter;
    mPulse2.mLengthCounter = bPulse2LCHalt ? 0 : mPulse2.mLengthCounter;
}


void APU::WriteFrameCounter(const uint8_t Data)
{
    mRegisters.FrameCounter = Data;
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
    uint8_t Value = mEnvelope.mValue;
    Value *= !mSweep.mbIsMutingChannel;
    Value *= mLengthCounter > 0;

    uint16_t Timer = EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::TIMER_LOW);
    Timer |= (LengthCounterRegister & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;

    // Sequencer Begin
    mSequencerIndex += (Timer / 2) > 0;

    uint8_t Duty = (TimerRegister & static_cast<uint8_t>(EPULSE_TIMER_MASKS::DUTY)) >> 6;
    uint8_t SequencerOutput = (DUTY_CYCLE_SEQUENCES[Duty] & (1 << (mSequencerIndex % 8))) != 0;
    Value *= SequencerOutput;

    Value *= Timer < 8 ? 0 : 1;
    // Sequencer End

    return Value;
}

void APU::PulseUnit::ClockEnvelope(uint8_t& TimerRegister)
{
    bool bConstantVolume = (TimerRegister & static_cast<uint8_t>(EPULSE_TIMER_MASKS::CONSTANT_VOLUME)) != 0;
    if (!bConstantVolume)
    {
        if (mEnvelope.mValue == 0)
        {
            bool bLoop = (TimerRegister & static_cast<uint8_t>(EPULSE_TIMER_MASKS::ENVELOPE_LOOP)) != 0;
            mEnvelope.mValue = bLoop ? 15 : 0;
        }
        else
        {
            uint8_t Rate = (TimerRegister & static_cast<uint8_t>(EPULSE_TIMER_MASKS::VOLUME_ENVELOPE)) & 0b1111;
            mEnvelope.mValue = mEnvelope.mValue >= Rate ? mEnvelope.mValue - Rate : 0;
        }
    }
}

void APU::PulseUnit::ClockSweep(MemoryMapper* RAM, uint8_t& LengthCounterRegister, uint8_t& EnvelopeRegister, uint8_t& SweepRegister)
{
    uint16_t Period = EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::TIMER_LOW);
    Period |= (LengthCounterRegister & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;

    mSweep.mbIsEnabled = SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_ENABLED);
    uint8_t ShiftCount = SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_SHIFT);

    int32_t Change = ShiftCount == 0 ? Period : 0;
    for (int i = 0; i < ShiftCount; i++)
    {
        Change |= Period & (1 << i);
    }
    Period = Period >> ShiftCount;

    bool bNegateFlag = (SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_NEGATE)) != 0;

    // Pulse1 uses ones compliment
    Change = bNegateFlag ? (~Change + mbIsPulse2) : Change;
    Period = std::max(Period + Change, 0);

    mSweep.mDivider -= 1;

    if (mSweep.mLastSweepRegister != SweepRegister)
    {
        mSweep.mLastSweepRegister = SweepRegister;

        mSweep.mDivider = (SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_PERIOD)) >> 4;
        mSweep.mDivider += 1;
    }

    mSweep.mbIsMutingChannel = Change > 0x7FF ? 0 : 1;
    mSweep.mbIsMutingChannel |= Period < 8 ? 0 : 1;

    if (!mSweep.mbIsMutingChannel)
    {
        if (mSweep.mbIsEnabled && (ShiftCount > 0))
        {
            EnvelopeRegister &= ~static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::TIMER_LOW);
            EnvelopeRegister |= Period & 0xFF;

            LengthCounterRegister &= ~static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH);
            LengthCounterRegister |= (Period >> 8);
        }
    }
}

void APU::PulseUnit::ClockLengthCounter(bool bInfinite)
{
    // Infinite doesn't modify it at all, so if it's at 0 it stays at zero.
    if (bInfinite)
        return;

    mLengthCounter = mLengthCounter > 0 ? mLengthCounter-- : 0;
}
