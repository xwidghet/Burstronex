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
        mNoise.mNoiseTimerLUT = {4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708,  944, 1890, 3778};
    }
    else
    {
        mFrameInterruptCycleCount = NTSC_FRAME_INTERRUPT_CYCLE_COUNT;
        mSequenceCycleInterval = NTSC_SEQUENCE_STEP_CYCLE_COUNT;
        mNoise.mNoiseTimerLUT = {4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068};
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
        mTriangle.ClockTimer(mRegisters.Triangle_Timer, mRegisters.Triangle_LengthCounter);

        //float NewSample = TestOutput(CPUCycles, i);
        float NewSample = DACOutput()*2.f - 1.f;

        mAudioBuffer.Push(NewSample);
    }
}

void APU::ExecuteCycle()
{
    mPulse1.ClockSequencer(mRegisters.Pulse1_Timer, mRegisters.Pulse1_LengthCounter);
    mPulse2.ClockSequencer(mRegisters.Pulse2_Timer, mRegisters.Pulse2_LengthCounter);

    mPulse1.Execute(mRegisters.Pulse1_Envelope);
    mPulse2.Execute(mRegisters.Pulse2_Envelope);

    mNoise.ClockTimer(mRegisters.Noise_ModePeriod);
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

    if ((mCyclesSinceFrameInterrupt % mSequenceCycleInterval) == 0)
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
    bool bPulse1Infinite = (mRegisters.Pulse1_Envelope & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::ENVELOPE_LOOP));
    bool bPulse2Infinite = (mRegisters.Pulse2_Envelope & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::ENVELOPE_LOOP));
    mPulse1.ClockLengthCounter(bPulse1Infinite, mRegisters.Pulse1_LengthCounter);
    mPulse2.ClockLengthCounter(bPulse2Infinite, mRegisters.Pulse1_LengthCounter);

    mPulse1.ClockSweep(mRegisters.Pulse1_Sweep);
    mPulse2.ClockSweep(mRegisters.Pulse2_Sweep);

    mTriangle.ClockLengthCounter(mRegisters.Triangle_LinearCounter);

    mNoise.ClockLengthCounter(mRegisters.Noise_Envelope);
}

void APU::QuarterFrame()
{
    // Clock Envelopes and Triangle linear counter.
    mPulse1.ClockEnvelope(mRegisters.Pulse1_Envelope);
    mPulse2.ClockEnvelope(mRegisters.Pulse2_Envelope);

    mTriangle.ClockLinearCounter(mRegisters.Triangle_LinearCounter);

    mNoise.ClockEnvelope(mRegisters.Noise_Envelope);
}

float APU::DACOutput()
{
    uint8_t Triangle = mTriangle.mOutputSample;
    uint8_t Noise = mNoise.mOutputSample;
    uint8_t DMC = 0;

    // Pulse, Triangle, and Noise should be 0-15
    // DMC should be 0-127
    // DAC Output is a float in the range [0.0, 1.0]
    float PulseOut = SquareLUT[mPulse1.mOutputSample + mPulse2.mOutputSample];

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

    uint16_t Timer = uint16_t(mRegisters.Pulse1_LengthCounter & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;
    Timer |= mRegisters.Pulse1_Timer;

    mPulse1.mSweep.mTargetPeriod = Timer + 1;
    mPulse1.mSweep.mPeriod = mPulse1.mSweep.mTargetPeriod;

    mPulse1.mSequencerIndex = 0;
    mPulse1.mEnvelope.mbReloadFlag = true;
    mPulse1.mSweep.mbReloadFlag = true;
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

    uint16_t Timer = uint16_t(mRegisters.Pulse2_LengthCounter & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;
    Timer |= mRegisters.Pulse2_Timer;

    mPulse2.mSweep.mTargetPeriod = Timer + 1;
    mPulse2.mSweep.mPeriod = mPulse2.mSweep.mTargetPeriod;

    mPulse2.mSequencerIndex = 0;

    mPulse2.mEnvelope.mbReloadFlag = true;
    mPulse2.mSweep.mbReloadFlag = true;
}

void APU::WriteTriangle_LinearCounter(const uint8_t Data)
{
    mRegisters.Triangle_LinearCounter = Data;
}

void APU::WriteTriangle_Timer(const uint8_t Data)
{
    mRegisters.Triangle_Timer = Data;
}

void APU::WriteTriangle_LengthCounter(const uint8_t Data)
{
    mRegisters.Triangle_LengthCounter = Data;
    mTriangle.mbLinearCounterReloadFlag = true;

    uint16_t Timer = uint16_t(mRegisters.Triangle_LengthCounter & static_cast<uint8_t>(ETRIANGLE_LENGTHCOUNTER_MASKS::TIMER_HIGH)) << 8;
    Timer |= mRegisters.Triangle_Timer + 1;

    mTriangle.mTimer = Timer;

    uint8_t LengthCounterIndex = (mRegisters.Triangle_LengthCounter & static_cast<uint8_t>(ETRIANGLE_LENGTHCOUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
    mTriangle.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];
}

void APU::WriteNoise_Envelope(const uint8_t Data)
{
    mRegisters.Noise_Envelope = Data;
}

void APU::WriteNoise_ModePeriod(const uint8_t Data)
{
    mRegisters.Noise_ModePeriod = Data;
}

void APU::WriteNoise_LengthCounter(const uint8_t Data)
{
    mRegisters.Noise_LengthCounter = Data;

    uint8_t LengthCounterIndex = (mRegisters.Noise_LengthCounter & static_cast<uint8_t>(ENOISE_LENGTHCOUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
    mNoise.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];

    mNoise.mEnvelope.mbReloadFlag = true;
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
    uint8_t Output = mRegisters.Status;

    if (mPulse1.mLengthCounter == 0)
        Output &= ~static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_1_PLAYING);
    if (mPulse2.mLengthCounter == 0)
        Output &= ~static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_2_PLAYING);
    if (mTriangle.mLengthCounter == 0)
        Output &= ~static_cast<uint8_t>(ESTATUS_WRITE_MASKS::TRIANGLE_PLAYING);
    if (mNoise.mLengthCounter == 0)
        Output &= ~static_cast<uint8_t>(ESTATUS_WRITE_MASKS::NOISE_PLAYING);

    // Clears Frame Interrupt flag but not DMC Interrupt flag
    mRegisters.Status &= ~static_cast<uint8_t>(ESTATUS_READ_MASKS::FRAME_INTERRUPT);

    // Are there multiple sources of IRQ?
    mCPU->SetIRQ(false);

    return Output;
}

void APU::WriteStatus(const uint8_t Data)
{
    mRegisters.Status = Data;

    // Todo: Implement all the other channels
    bool bPulse1LCHalt = !(mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_1_PLAYING));
    bool bPulse2LCHalt = !(mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_2_PLAYING));
    bool bTrianglePlaying = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::TRIANGLE_PLAYING));

    mPulse1.mLengthCounter = bPulse1LCHalt ? 0 : mPulse1.mLengthCounter;
    mPulse2.mLengthCounter = bPulse2LCHalt ? 0 : mPulse2.mLengthCounter;

    mTriangle.mbIsEnabled = bTrianglePlaying;
    mTriangle.mLengthCounter = bTrianglePlaying ? mTriangle.mLengthCounter : 0;
}


void APU::WriteFrameCounter(const uint8_t Data)
{
    mRegisters.FrameCounter = Data;
    mCyclesSinceFrameInterrupt = 0;
    mSequenceIndex = 0;
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

void APU::PulseUnit::Execute(const uint8_t EnvelopeRegister)
{
    uint8_t Value = 1;
    Value *= !mSweep.mbIsMutingChannel;
    Value *= mLengthCounter > 0;

    uint8_t Duty = (EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::DUTY)) >> 6;
    uint8_t SequencerOutput = (DUTY_CYCLE_SEQUENCES[Duty] & (1 << (7 - mSequencerIndex))) != 0;
    Value *= SequencerOutput;

    // Target period?
    Value *= mSweep.mPeriod < 8 ? 0 : 1;
    // Sequencer End

    mOutputSample = Value*mEnvelope.mValue;
}

void APU::PulseUnit::ClockSequencer(const uint8_t TimerRegister, const uint8_t LengthCounterRegister)
{
    // Sequencer Begin
    if (mSweep.mPeriod == 0)
    {
        mSequencerIndex = mSequencerIndex < 7 ? mSequencerIndex + 1 : 0;

        uint16_t Timer = uint16_t(LengthCounterRegister & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;
        Timer |= TimerRegister;
        mSweep.mPeriod = Timer + 1;
    }

    if (mSweep.mPeriod > 0)
    {
        mSweep.mPeriod -= 1;
    }
}

void APU::PulseUnit::ClockEnvelope(const uint8_t EnvelopeRegister)
{
    if (mEnvelope.mbReloadFlag)
    {
        mEnvelope.mbReloadFlag = false;

        mEnvelope.mValue = 15;

        mEnvelope.mDivider = (EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::VOLUME_ENVELOPE)) & 0b1111;
        mEnvelope.mDivider += 1;
    }
    else
    {
        if (mEnvelope.mDivider == 0)
        {
            bool bLoop = (EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::ENVELOPE_LOOP)) != 0;
            if (bLoop && mEnvelope.mValue == 0)
            {
                mEnvelope.mValue = 15;
            }
            else if (mEnvelope.mValue > 0)
            {
                mEnvelope.mValue -= 1;
            }

            mEnvelope.mDivider = (EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::VOLUME_ENVELOPE)) & 0b1111;
            mEnvelope.mDivider += 1;
        }
        else
        {
            mEnvelope.mDivider -= 1;
        }
    }

    bool bConstantVolume = (EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::CONSTANT_VOLUME)) != 0;
    if (bConstantVolume)
    {
        mEnvelope.mValue = (EnvelopeRegister & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::VOLUME_ENVELOPE)) & 0b1111;
    }
}

void APU::PulseUnit::ClockSweep(const uint8_t SweepRegister)
{
    // Period always iterates, but only writes back the new value when not muted, is enabled, and shift count is non-zero.
    mSweep.mbIsEnabled = SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_ENABLED);
    uint8_t ShiftCount = SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_SHIFT);
    bool bNegateFlag = (SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_NEGATE)) != 0;

    // Pulse1 uses ones compliment
    uint16_t Change = mSweep.mPeriod >> ShiftCount;
    mSweep.mTargetPeriod = bNegateFlag ? (mSweep.mPeriod + ~Change  + mbIsPulse2) : (mSweep.mPeriod + Change);

    mSweep.mbIsMutingChannel = mSweep.mTargetPeriod > 0x7FF ? 1 : 0;
    mSweep.mbIsMutingChannel |= mSweep.mPeriod < 8 ? 1 : 0;

    if (mSweep.mDivider > 0)
    {
        mSweep.mDivider -= 1;

        // Does Sweep Divider clock the sweep unit only when it hits zero like the envelope?
        if (mSweep.mDivider == 0)
        {

            if (mSweep.mbIsEnabled && ShiftCount > 0)
            {
                if (!mSweep.mbIsMutingChannel)
                {
                    mSweep.mPeriod = mSweep.mTargetPeriod;
                }
                else
                {
                    mSweep.mDivider = (SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_PERIOD)) >> 4;
                    mSweep.mDivider += 1;
                }
            }
        }
    }

    if (mSweep.mbReloadFlag)
    {
        mSweep.mbReloadFlag = false;

        mSweep.mDivider = (SweepRegister & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_PERIOD)) >> 4;
        mSweep.mDivider += 1;
    }
}

void APU::PulseUnit::ClockLengthCounter(bool bInfinite, uint8_t LengthCounter)
{
    // Infinite doesn't modify it at all, so if it's at 0 it stays at zero.
    if (bInfinite)
        return;

    if (mLengthCounter > 0)
    {
        mLengthCounter -= 1;
    }
}

void APU::TriangleUnit::ClockTimer(const uint8_t TimerRegister, const uint8_t LengthCounterRegister)
{
    if (mTimer == 0)
    {
        if (mLinearCounter > 0 && mLengthCounter > 0)
        {
            ClockSequencer();
        }

        uint16_t Timer = uint16_t(LengthCounterRegister & static_cast<uint8_t>(ETRIANGLE_LENGTHCOUNTER_MASKS::TIMER_HIGH)) << 8;
        Timer |= TimerRegister;

        mTimer = Timer + 1;
    }
    else if (mTimer > 0)
    {
        mTimer -= 1;
    }

}

void APU::TriangleUnit::ClockLengthCounter(const uint8_t LinearCounterRegister)
{
    bool bHalt = (LinearCounterRegister & static_cast<uint8_t>(ETRIANGLE_LINEAR_COUNTER_MASKS::LENGTH_COUNTER_HALT_LINEAR_COUNTER_CONTROL)) != 0;
    if (bHalt)
        return;

    if (mLengthCounter > 0)
    {
        mLengthCounter -= 1;
    }
}

void APU::TriangleUnit::ClockLinearCounter(const uint8_t LinearCounterRegister)
{
    bool bHalt = (LinearCounterRegister & static_cast<uint8_t>(ETRIANGLE_LINEAR_COUNTER_MASKS::LENGTH_COUNTER_HALT_LINEAR_COUNTER_CONTROL)) != 0;

    if (mbLinearCounterReloadFlag)
    {
        mLinearCounter = LinearCounterRegister & static_cast<uint8_t>(ETRIANGLE_LINEAR_COUNTER_MASKS::LINEAR_COUNTER_LOAD);
    }
    else
    {
        if (mLinearCounter > 0)
            mLinearCounter -= 1;
    }

    if (bHalt == false)
    {
        mbLinearCounterReloadFlag = false;
    }
}

void APU::TriangleUnit::ClockSequencer()
{
    mSequenceIndex = mSequenceIndex < 31 ? mSequenceIndex + 1 : 0;
    mOutputSample = mbIsEnabled * TRIANGLE_SEQUENCE_TABLE[mSequenceIndex];
}

void APU::NoiseUnit::ClockTimer(const uint8_t ModePeriodRegister)
{
    if (mTimer == 0)
    {
        uint8_t TimerIndex = ModePeriodRegister & static_cast<uint8_t>(ENOISE_MODE_PERIOD_MASKS::NOISE_PERIOD);
        mTimer = mNoiseTimerLUT[TimerIndex];

        ClockSequencer(ModePeriodRegister);
    }
    else
    {
        mTimer -= 1;
    }
}

void APU::NoiseUnit::ClockEnvelope(const uint8_t EnvelopeRegister)
{
    if (mEnvelope.mbReloadFlag)
    {
        mEnvelope.mbReloadFlag = false;

        mEnvelope.mValue = 15;

        mEnvelope.mDivider = (EnvelopeRegister & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::VOLUME_ENVELOPE)) & 0b1111;
        mEnvelope.mDivider += 1;
    }
    else
    {
        if (mEnvelope.mDivider == 0)
        {
            bool bLoop = (EnvelopeRegister & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::ENVELOPE_LOOP_LENGTH_COUNTER_HALT)) != 0;
            if (bLoop && mEnvelope.mValue == 0)
            {
                mEnvelope.mValue = 15;
            }
            else if (mEnvelope.mValue > 0)
            {
                mEnvelope.mValue -= 1;
            }

            mEnvelope.mDivider = (EnvelopeRegister & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::VOLUME_ENVELOPE)) & 0b1111;
            mEnvelope.mDivider += 1;
        }
        else
        {
            mEnvelope.mDivider -= 1;
        }
    }

    bool bConstantVolume = (EnvelopeRegister & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::CONSTANT_VOLUME)) != 0;
    if (bConstantVolume)
    {
        mEnvelope.mValue = (EnvelopeRegister & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::VOLUME_ENVELOPE)) & 0b1111;
    }
}

void APU::NoiseUnit::ClockLengthCounter(const uint8_t EnvelopeRegister)
{
    bool bHalt = (EnvelopeRegister & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::ENVELOPE_LOOP_LENGTH_COUNTER_HALT)) != 0;
    if (bHalt)
        return;

    if (mLengthCounter > 0)
    {
        mLengthCounter -= 1;
    }
}

void APU::NoiseUnit::ClockSequencer(const uint8_t ModePeriodRegister)
{
    bool bMode = (ModePeriodRegister & static_cast<uint8_t>(ENOISE_MODE_PERIOD_MASKS::NOISE_MODE)) != 0;
    bool bBit0 = mLFSR & 0b1;
    bool bBit1 = bMode ? (mLFSR & 0b01000000) >> 6 : (mLFSR & 0b10) >> 1;

    bool bFeedback = bBit0 ^ bBit1;
    mLFSR = mLFSR >> 1;
    mLFSR |= uint16_t(bFeedback) << 14;

    bBit0 = mLFSR & 0b1;

    bool bOutputValue = !bBit0 && mLengthCounter > 0;
    mOutputSample = bOutputValue ? mEnvelope.mValue : 0;
}
