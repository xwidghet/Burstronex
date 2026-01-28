#include "APU.h"

#include "CPU.h"
#include "CircularBuffer.h"
#include "Logger.h"
#include "MemoryMapper.h"
#include "RomParameters.h"
#include "StatisticsManager.h"

#include "miniaudio.c"

#include <algorithm>
#include <cstring>

constexpr std::array<float, 31> GenerateSquareTable()
{
    std::array<float, 31> SquareTable = {};
    for (int i = 1; i < 31; i++)
    {
        SquareTable[i] = 95.52 / (8128.0 / double(i) + 100);
    }

    return SquareTable;
}

constexpr std::array<float, 203>  GenerateTND()
{
    std::array<float, 203> TNDTable = {};
    for (int i = 1; i < 203; i++)
    {
        TNDTable[i] = 163.67 / (24329.0 / double(i) + 100);
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

    mMode = 0;
    mbCycleToggle = false;
    mCyclesSinceFrameInterrupt = 0;
    mFrameInterruptCycleCount = 0;
    mSequenceCycleCount = 0;

    if (Timing == ECPU_TIMING::PAL)
    {
        mFrameInterruptCycleCount = PAL_FRAME_INTERRUPT_CYCLE_COUNT;
        mSequenceCycleInterval = PAL_SEQUENCE_STEP_CYCLE_COUNT;
        mNoise.mNoiseTimerLUT = {4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708,  944, 1890, 3778};
        mDMC.mRateTable = {398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118,  98,  78,  66,  50 };

    }
    else
    {
        mFrameInterruptCycleCount = NTSC_FRAME_INTERRUPT_CYCLE_COUNT;
        mSequenceCycleInterval = NTSC_SEQUENCE_STEP_CYCLE_COUNT;
        mNoise.mNoiseTimerLUT = {4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068};
        mDMC.mRateTable = {428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106,  84,  72,  54};
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
    // If Burstronex starts first: Behavior is of exclusive mode, other applications can't play audio.
    // If Burstronex starts second: Other applications behave normally, but Burstronex only outputs corrupted audio with cuts.
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
    for (int i = 0; i < CPUCycles; i++)
    {
        // Frame Counter Register write side-effect
        if (mFrameResetCycles > 0)
        {
            mFrameResetCycles -= 1;
            if (mFrameResetCycles == 0)
            {
                mSequenceCycleCount = 0;
            }
        }

        if (mbCycleToggle)
        {
            ExecuteCycle();
            ExecuteSequencer();
        }
        mbCycleToggle = !mbCycleToggle;

        // Every CPU Cycle the APU outputs one sample,
        // and the Triangle unit clocks on every CPU Cycle
        mTriangle.ClockTimer();

        //float NewSample = TestOutput(CPUCycles, i);
        float NewSample = DACOutput()*2.f - 1.f;

        mAudioBuffer.Push(NewSample);
        mCyclesSinceFrameInterrupt++;
    }
}

void APU::ExecuteCycle()
{
    mPulse1.ClockSweep();
    mPulse2.ClockSweep();

    mPulse1.ClockSequencer();
    mPulse2.ClockSequencer();

    mPulse1.Execute();
    mPulse2.Execute();

    mNoise.ClockTimer();

    mDMC.ClockMemoryReader(mCPU, mRAM);
    mDMC.ClockOutputUnit();
}

void APU::ExecuteSequencer()
{
    if (mMode == 0)
        ExecuteMode0Sequencer();
    else
        ExecuteMode1Sequencer();
}

void APU::ExecuteMode0Sequencer()
{
    if ((mSequenceCycleCount % mSequenceCycleInterval) == 0)
    {
        switch (mSequenceCycleCount / mSequenceCycleInterval)
        {
            case 0:
                break;
            case 1:
                QuarterFrame();
                break;
            case 2:
                HalfFrame();
                QuarterFrame();
                break;
            case 3:
                QuarterFrame();
                break;
            case 4:
                HalfFrame();
                QuarterFrame();
                TriggerFrameInterrupt();
                break;
        }
    }

    if (mSequenceCycleCount == (mSequenceCycleInterval * 4))
    {
        mSequenceCycleCount = 0;
    }
    else
    {
        mSequenceCycleCount++;
    }
}

void APU::ExecuteMode1Sequencer()
{
    if ((mSequenceCycleCount % mSequenceCycleInterval) == 0)
    {
        switch (mSequenceCycleCount / mSequenceCycleInterval)
        {
            case 0:
                HalfFrame();
                QuarterFrame();
                break;
            case 1:
                QuarterFrame();
                break;
            case 2:
                HalfFrame();
                QuarterFrame();
                break;
            case 3:
                QuarterFrame();
                break;
            case 4:
                // Nothing Happens (TM?)
                break;
        }
    }

    if (mSequenceCycleCount == (mSequenceCycleInterval * 5))
    {
        mSequenceCycleCount = 0;
    }
    else
    {
        mSequenceCycleCount++;
    }
}

void APU::HalfFrame()
{
    mPulse1.ClockLengthCounter();
    mPulse2.ClockLengthCounter();

    mPulse1.ClockHalfFrameSweep();
    mPulse2.ClockHalfFrameSweep();

    mTriangle.ClockLengthCounter();

    mNoise.ClockLengthCounter();
}

void APU::QuarterFrame()
{
    mPulse1.mEnvelope.Clock();
    mPulse2.mEnvelope.Clock();
    mNoise.mEnvelope.Clock();

    mTriangle.ClockLinearCounter();
}

void APU::TriggerFrameInterrupt()
{
    mCyclesSinceFrameInterrupt = 0;
    mAudioFrameCount++;

    bool bBlockIRQ = (mRegisters.FrameCounter & static_cast<uint8_t>(EFRAME_COUNTER_MASKS::IRQ_INHIBIT)) != 0;

    if (mMode == 0 && !bBlockIRQ)
    {
        mRegisters.Status |= static_cast<uint8_t>(ESTATUS_READ_MASKS::FRAME_INTERRUPT);
        mCPU->SetFrameCounterIRQ(true);
    }

    // Start audio device once we have one audio frame of data generated.
    if (!mbStartedDevice)
    {
        mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::INFO, "Started Audio Device\n");
        ma_device_start(&*mAudioDevice);
        mbStartedDevice = true;
    }
}

float APU::DACOutput()
{
    // Pulse, Triangle, and Noise should be 0-15
    // DMC should be 0-127
    // DAC Output is a float in the range [0.0, 1.0]
    float PulseOut = SquareLUT[mPulse1.mOutputSample + mPulse2.mOutputSample];

    float TNDOut = TNDLUT[3*mTriangle.mOutputSample + 2*mNoise.mOutputSample + mDMC.mOutputSample];

    return PulseOut + TNDOut;
}

float APU::GetBufferFillPercentage() const
{
    if (mbStartedDevice)
        return mAudioBuffer.GetPercentageFilled();
    else
        return 1.f;
}

void APU::WritePulse1_Envelope(const uint8_t Data)
{
    mPulse1.mDuty = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::DUTY)) >> 6;
    mPulse1.mbLengthCounterHalt = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::ENVELOPE_LOOP)) != 0;
    mPulse1.mEnvelope.mbIsLooping = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::ENVELOPE_LOOP)) != 0;
    mPulse1.mEnvelope.mbIsConstantVolume = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::CONSTANT_VOLUME)) != 0;
    mPulse1.mEnvelope.mConstantVolume_Envelope = (Data& static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::VOLUME_ENVELOPE));
}

void APU::WritePulse1_Sweep(const uint8_t Data)
{
    mPulse1.mSweep.mShiftCount = Data & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_SHIFT);
    mPulse1.mSweep.mbNegateFlag = (Data & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_NEGATE)) != 0;
    mPulse1.mSweep.mDividerLength = (Data & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_PERIOD)) >> 4;

    mPulse1.mSweep.mbIsEnabled = (Data & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_ENABLED)) != 0;
    mPulse1.mSweep.mbReloadFlag = true;
}

void APU::WritePulse1_Timer(const uint8_t Data)
{
    mPulse1.mSweep.mPeriodLength &= 0xFF00;
    mPulse1.mSweep.mPeriodLength |=  Data;
}

void APU::WritePulse1_LengthCounter(const uint8_t Data)
{
    bool bPulse1LCHalt = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_1_PLAYING)) == 0;
    if (!bPulse1LCHalt)
    {
        uint8_t LengthCounterIndex = (Data & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
        mPulse1.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];
    }

    mPulse1.mSequencerIndex = 0;
    mPulse1.mEnvelope.mbReloadFlag = true;

    int16_t TimerHigh = uint16_t(Data & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;
    mPulse1.mSweep.mPeriodLength &= 0x00FF;
    mPulse1.mSweep.mPeriodLength |= TimerHigh;
}

void APU::WritePulse2_Envelope(const uint8_t Data)
{
    mPulse2.mDuty = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::DUTY)) >> 6;
    mPulse2.mbLengthCounterHalt = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::ENVELOPE_LOOP)) != 0;
    mPulse2.mEnvelope.mbIsLooping = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::ENVELOPE_LOOP)) != 0;
    mPulse2.mEnvelope.mbIsConstantVolume = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::CONSTANT_VOLUME)) != 0;
    mPulse2.mEnvelope.mConstantVolume_Envelope = (Data & static_cast<uint8_t>(EPULSE_ENVELOPE_MASKS::VOLUME_ENVELOPE));
}

void APU::WritePulse2_Sweep(const uint8_t Data)
{
    mPulse2.mSweep.mShiftCount = Data & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_SHIFT);
    mPulse2.mSweep.mbNegateFlag = (Data & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_NEGATE)) != 0;
    mPulse2.mSweep.mDividerLength = (Data & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_PERIOD)) >> 4;

    mPulse2.mSweep.mbIsEnabled = (Data & static_cast<uint8_t>(EPULSE_SWEEP_MASKS::SWEEP_UNIT_ENABLED)) != 0;
    mPulse2.mSweep.mbReloadFlag = true;
}

void APU::WritePulse2_Timer(const uint8_t Data)
{
    mPulse2.mSweep.mPeriodLength &= 0xFF00;
    mPulse2.mSweep.mPeriodLength |=  Data;
}

void APU::WritePulse2_LengthCounter(const uint8_t Data)
{
    bool bPulse2LCHalt = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_2_PLAYING)) == 0;
    if (!bPulse2LCHalt)
    {
        uint8_t LengthCounterIndex = (Data & static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
        mPulse2.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];
    }

    mPulse2.mSequencerIndex = 0;
    mPulse2.mEnvelope.mbReloadFlag = true;

    int16_t TimerHigh = uint16_t(Data& static_cast<uint8_t>(EPULSE_LENGTH_COUNTER_MASKS::TIMER_HIGH)) << 8;
    mPulse2.mSweep.mPeriodLength &= 0x00FF;
    mPulse2.mSweep.mPeriodLength |= TimerHigh;
}

void APU::WriteTriangle_LinearCounter(const uint8_t Data)
{
    mTriangle.mbLengthCounterHalt_LinearCounterControl = (Data & static_cast<uint8_t>(ETRIANGLE_LINEAR_COUNTER_MASKS::LENGTH_COUNTER_HALT_LINEAR_COUNTER_CONTROL)) != 0;
    mTriangle.mLinearCounterLength = Data & static_cast<uint8_t>(ETRIANGLE_LINEAR_COUNTER_MASKS::LINEAR_COUNTER_LOAD);
}

void APU::WriteTriangle_Timer(const uint8_t Data)
{
    mTriangle.mTimerLength &= 0xFF00;
    mTriangle.mTimerLength |= Data;
}

void APU::WriteTriangle_LengthCounter(const uint8_t Data)
{
    bool bTriangleLCHalt = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::TRIANGLE_PLAYING)) == 0;
    if (!bTriangleLCHalt)
    {
        uint8_t LengthCounterIndex = (Data & static_cast<uint8_t>(ETRIANGLE_LENGTHCOUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
        mTriangle.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];
    }

    uint16_t TimerHigh = uint16_t(Data & static_cast<uint8_t>(ETRIANGLE_LENGTHCOUNTER_MASKS::TIMER_HIGH)) << 8;

    mTriangle.mTimerLength &= 0x00FF;
    mTriangle.mTimerLength |= TimerHigh;

    mTriangle.mbLinearCounterReloadFlag = true;

}

void APU::WriteNoise_Envelope(const uint8_t Data)
{
    mNoise.mEnvelope.mbIsLooping = (Data & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::ENVELOPE_LOOP_LENGTH_COUNTER_HALT)) != 0;
    mNoise.mbLengthCounterHalt = (Data & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::ENVELOPE_LOOP_LENGTH_COUNTER_HALT)) != 0;
    mNoise.mEnvelope.mbIsConstantVolume = (Data & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::CONSTANT_VOLUME)) != 0;
    mNoise.mEnvelope.mConstantVolume_Envelope = (Data & static_cast<uint8_t>(ENOISE_ENVELOPE_MASKS::VOLUME_ENVELOPE));
}

void APU::WriteNoise_ModePeriod(const uint8_t Data)
{
    uint8_t TimerIndex = Data & static_cast<uint8_t>(ENOISE_MODE_PERIOD_MASKS::NOISE_PERIOD);
    mNoise.mTimerLength = mNoise.mNoiseTimerLUT[TimerIndex];

    mNoise.mbMode = (Data & static_cast<uint8_t>(ENOISE_MODE_PERIOD_MASKS::NOISE_MODE)) != 0;
}

void APU::WriteNoise_LengthCounter(const uint8_t Data)
{
    bool bNoiseLCHalt = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::NOISE_PLAYING)) == 0;
    if (!bNoiseLCHalt)
    {
        uint8_t LengthCounterIndex = (Data & static_cast<uint8_t>(ENOISE_LENGTHCOUNTER_MASKS::LENGTH_COUNTER_LOAD)) >> 3;
        mNoise.mLengthCounter = LENGTH_COUNTER_TABLE[LengthCounterIndex];
    }

    mNoise.mEnvelope.mbReloadFlag = true;
}

void APU::WriteDMC_ILR(const uint8_t Data)
{
    mDMC.mbIsIRQEnabled = (Data & static_cast<uint8_t>(EDMC_ILR_MASKS::IRQ_ENABLE)) != 0;
    mDMC.mbIsLooping = (Data & static_cast<uint8_t>(EDMC_ILR_MASKS::LOOP)) != 0;
    mDMC.mRate = mDMC.mRateTable[Data & static_cast<uint8_t>(EDMC_ILR_MASKS::FREQUENCY)];

    if (!mDMC.mbIsIRQEnabled)
    {
        mCPU->SetDMCIRQ(false);
    }
}

void APU::WriteDMC_LoadCounter(const uint8_t Data)
{
    mDMC.mOutputSample = Data & static_cast<uint8_t>(EDMC_LOADCOUNTER_MASKS::LOAD_COUNTER);
}

void APU::WriteDMC_SampleAddress(const uint8_t Data)
{
    mDMC.mSampleAddress = 0xC000 + (uint16_t(Data) << 6);
    mDMC.mCurrentSampleAddress = mDMC.mSampleAddress;
}

void APU::WriteDMC_SampleLength(const uint8_t Data)
{
    mDMC.mSampleLength = (uint16_t(Data) << 4) + 1;
}

uint8_t APU::ReadStatus()
{
    uint8_t Output = 0;

    Output |= mPulse1.mLengthCounter > 0 ? static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_1_PLAYING) : 0;
    Output |= mPulse2.mLengthCounter > 0 ? static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_2_PLAYING) : 0;
    Output |= mTriangle.mLengthCounter > 0 ? static_cast<uint8_t>(ESTATUS_WRITE_MASKS::TRIANGLE_PLAYING) : 0;
    Output |= mNoise.mLengthCounter > 0 ? static_cast<uint8_t>(ESTATUS_WRITE_MASKS::NOISE_PLAYING) : 0;
    Output |= mDMC.mBytesRemaining > 0 ? static_cast<uint8_t>(ESTATUS_WRITE_MASKS::DMC_ACTIVE) : 0;

    // Clears Frame Interrupt flag but not DMC Interrupt flag
    mRegisters.Status &= ~static_cast<uint8_t>(ESTATUS_READ_MASKS::FRAME_INTERRUPT);

    // DMC IRQ is NOT cleared here.
    mCPU->SetFrameCounterIRQ(false);

    return Output;
}

void APU::WriteStatus(const uint8_t Data)
{
    mRegisters.Status = Data;

    // Todo: Implement all the other channels
    bool bPulse1LCHalt = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_1_PLAYING)) == 0;
    bool bPulse2LCHalt = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::PULSE_2_PLAYING)) == 0;
    bool bTrianglePlaying = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::TRIANGLE_PLAYING));
    bool bDMCPlaying = (mRegisters.Status & static_cast<uint8_t>(ESTATUS_WRITE_MASKS::DMC_ACTIVE));

    mPulse1.mLengthCounter = bPulse1LCHalt ? 0 : mPulse1.mLengthCounter;
    mPulse2.mLengthCounter = bPulse2LCHalt ? 0 : mPulse2.mLengthCounter;

    mTriangle.mbIsEnabled = bTrianglePlaying;
    mTriangle.mLengthCounter = bTrianglePlaying ? mTriangle.mLengthCounter : 0;

    if (bDMCPlaying)
    {
        mDMC.mBytesRemaining = mDMC.mBytesRemaining == 0 ? mDMC.mSampleLength : mDMC.mBytesRemaining;
    }
    else
    {
        mDMC.mBytesRemaining = 0;
    }

    mCPU->SetDMCIRQ(false);
}


void APU::WriteFrameCounter(const uint8_t Data)
{
    mRegisters.FrameCounter = Data;
    mMode = (mRegisters.FrameCounter & static_cast<uint8_t>(EFRAME_COUNTER_MASKS::MODE)) != 0;

    // Last sequence is triggered 2-3 CPU cycles after Frame Counter is written, on the next even cycle.
    mFrameResetCycles = 3;
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

void APU::EnvelopeUnit::Clock()
{
    if (mbReloadFlag)
    {
        mbReloadFlag = false;

        mValue = 15;

        mDivider = mConstantVolume_Envelope;
    }
    else
    {
        if (mDivider == 0)
        {
            if (mbIsLooping && mValue == 0)
            {
                mValue = 15;
            }
            else if (mValue > 0)
            {
                mValue -= 1;
            }

            mDivider = mConstantVolume_Envelope;
        }
        else
        {
            mDivider -= 1;
        }
    }
}

uint8_t APU::EnvelopeUnit::GetVolume() const
{
    return mbIsConstantVolume ? mConstantVolume_Envelope : mValue;
}

void APU::PulseUnit::Execute()
{
    uint8_t Value = 1;
    Value *= !mSweep.mbIsMutingChannel;
    Value *= mLengthCounter > 0;

    uint8_t SequencerOutput = (DUTY_CYCLE_SEQUENCES[mDuty] & (1 << mSequencerIndex)) != 0;
    Value *= SequencerOutput;

    // 12.4Khz frequency limit.
    Value *= mSweep.mPeriodLength >= 8;

    mOutputSample = Value*mEnvelope.GetVolume();
}

void APU::PulseUnit::ClockSequencer()
{
    if (mSweep.mPeriod == 0)
    {
        mSequencerIndex = (mSequencerIndex - 1) & 7;
        mSweep.mPeriod = mSweep.mPeriodLength;
    }
    else
    {
        mSweep.mPeriod -= 1;
    }
}

void APU::PulseUnit::ClockHalfFrameSweep()
{
    if (mSweep.mDivider == 0)
    {
        // Period always iterates, but only writes back the new value when not muted, is enabled, and shift count is non-zero.
        if (mSweep.mbIsEnabled && mSweep.mShiftCount > 0)
        {
            if (!mSweep.mbIsMutingChannel)
            {
                mSweep.mPeriodLength = mSweep.mTargetPeriod;
            }
        }
    }

    if (mSweep.mbReloadFlag || mSweep.mDivider == 0)
    {
        mSweep.mbReloadFlag = false;

        mSweep.mDivider = mSweep.mDividerLength;
    }
    else
    {
        mSweep.mDivider -= 1;
    }
}

void APU::PulseUnit::ClockSweep()
{
    // Pulse1 uses ones compliment
    uint16_t Change = (mSweep.mPeriodLength >> mSweep.mShiftCount);
    mSweep.mTargetPeriod = mSweep.mbNegateFlag ? (mSweep.mPeriodLength + ~Change + mbIsPulse2) : (mSweep.mPeriodLength + Change);

    mSweep.mbIsMutingChannel = mSweep.mTargetPeriod > 0x7FF;
    mSweep.mbIsMutingChannel |= mSweep.mPeriodLength < 8;
}

void APU::PulseUnit::ClockLengthCounter()
{
    // Halt doesn't modify it at all, so if it's at 0 it stays at zero.
    if (mbLengthCounterHalt)
        return;

    if (mLengthCounter > 0)
    {
        mLengthCounter -= 1;
    }
}

void APU::TriangleUnit::ClockTimer()
{
    if (mTimer == 0)
    {
        if (mLinearCounter > 0 && mLengthCounter > 0)
        {
            ClockSequencer();
        }

        mTimer = mTimerLength;
    }
    else if (mTimer > 0)
    {
        mTimer -= 1;
    }

}

void APU::TriangleUnit::ClockLengthCounter()
{
    if (mbLengthCounterHalt_LinearCounterControl)
        return;

    if (mLengthCounter > 0)
    {
        mLengthCounter -= 1;
    }
}

void APU::TriangleUnit::ClockLinearCounter()
{
    if (mbLinearCounterReloadFlag)
    {
        mLinearCounter = mLinearCounterLength;
    }
    else
    {
        if (mLinearCounter > 0)
            mLinearCounter -= 1;
    }

    if (mbLengthCounterHalt_LinearCounterControl == false)
    {
        mbLinearCounterReloadFlag = false;
    }
}

void APU::TriangleUnit::ClockSequencer()
{
    mSequenceIndex = mSequenceIndex < 31 ? mSequenceIndex + 1 : 0;
    mOutputSample = mbIsEnabled * TRIANGLE_SEQUENCE_TABLE[mSequenceIndex];
}

void APU::NoiseUnit::ClockTimer()
{
    if (mTimer == 0)
    {
        mTimer = mTimerLength;

        ClockSequencer();
    }
    else
    {
        mTimer -= 1;
    }
}

void APU::NoiseUnit::ClockLengthCounter()
{
    if (mbLengthCounterHalt)
        return;

    if (mLengthCounter > 0)
    {
        mLengthCounter -= 1;
    }
}

void APU::NoiseUnit::ClockSequencer()
{
    bool bBit0 = mLFSR & 0b1;
    bool bBit1 = mbMode ? (mLFSR & 0b01000000) >> 6 : (mLFSR & 0b10) >> 1;

    bool bFeedback = bBit0 ^ bBit1;
    mLFSR = mLFSR >> 1;
    mLFSR |= uint16_t(bFeedback) << 14;

    bBit0 = mLFSR & 0b1;

    bool bOutputValue = !bBit0 && mLengthCounter > 0;
    mOutputSample = bOutputValue ? mEnvelope.GetVolume() : 0;
}

void APU::DMCUnit::ClockMemoryReader(CPU* CPU, MemoryMapper* RAM)
{
    if (mSampleBuffer == 0 && mBytesRemaining > 0)
    {
        // Should stall the CPU for 1-4 cycles.
        mSampleBuffer = RAM->Read8Bit(mCurrentSampleAddress);

        mCurrentSampleAddress = mCurrentSampleAddress == 0xFFFF ? 0x8000 : mCurrentSampleAddress+1;

        mBytesRemaining -= 1;
        if (mBytesRemaining == 0)
        {
            if (mbIsLooping)
            {
                mCurrentSampleAddress = mSampleAddress;
                mBytesRemaining = mSampleLength;
            }
            else if (mbIsIRQEnabled)
            {
                CPU->SetDMCIRQ(true);
            }
        }
    }
}

void APU::DMCUnit::ClockOutputUnit()
{
    if (mTimer == 0)
    {
        mTimer = mRate;
    }
    else if (mTimer >= 2)
    {
        // Timer is in CPU cycles
        mTimer -= 2;

        if (mTimer == 0)
        {
            mTimer = mRate;

            if (!mbIsSilenced)
            {
                bool bPositive = ((mRSR & 0b1) == 1);
                if (bPositive && mOutputSample < 126)
                    mOutputSample += 2;

                if (!bPositive && mOutputSample > 1)
                    mOutputSample -= 2;
            }

            mRSR = mRSR >> 1;
            mBitsRemaining = mBitsRemaining > 0 ? mBitsRemaining-1 : 0;

            if (const bool bOutputCycleFinished = mBitsRemaining == 0)
            {
                mBitsRemaining = 8;
                mbIsSilenced = mSampleBuffer == 0 ? true : false;
                if (!mbIsSilenced)
                {
                    mRSR = mSampleBuffer;
                    mSampleBuffer = 0;
                }

            }
        }
    }
}
