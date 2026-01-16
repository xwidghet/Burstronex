#include "APU.h"

#include "CPU.h"
#include "CircularBuffer.h"
#include "Logger.h"
#include "MemoryMapper.h"
#include "RomParameters.h"

#include "miniaudio.c"

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
    auto& Buffer = *reinterpret_cast<CircularBuffer<float, NTSC_FRAME_INTERRUPT_CYCLE_COUNT*16>*>(pDevice->pUserData);
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
    mAudioContext = std::make_unique<ma_context>();
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

    mDownsampleRatio = CPU->GetClockFrequency() / TARGET_SAMPLE_RATE;
    bStartedDevice = false;

    std::memset(&mRegisters, 0, sizeof(mRegisters));

    InitAudio();
}

void APU::InitAudio()
{
    ma_context_config contextConfig = ma_context_config_init();
    contextConfig.alsa = {true};

    ma_backend backends[] = {
        ma_backend_alsa,
        ma_backend_pulseaudio,
        ma_backend_wasapi,
        ma_backend_dsound
    };

    if (ma_context_init(backends, 3, NULL, &*mAudioContext) != MA_SUCCESS) {
        mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::ERROR, "Failed to initialize miniaudio context\n");
    }

    ma_device_info* pPlaybackInfos;
    ma_uint32 playbackCount;
    ma_device_info* pCaptureInfos;
    ma_uint32 captureCount;
    if (ma_context_get_devices(&*mAudioContext, &pPlaybackInfos, &playbackCount, &pCaptureInfos, &captureCount) != MA_SUCCESS) {
        mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::ERROR, "Failed to get miniaudio devices\n");
    }

    for (ma_uint32 iDevice = 0; iDevice < playbackCount; iDevice += 1) {
        mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::ERROR, "{0} - {1}\n", iDevice, pPlaybackInfos[iDevice].name);
    }

    ma_device_config config = ma_device_config_init(ma_device_type_playback);

    // Hardcoded my device since ALSA refuses to use the correct device.
    config.playback.pDeviceID = &pPlaybackInfos[17].id;
    config.playback.format   = ma_format_f32;   // Set to ma_format_unknown to use the device's native format.
    config.playback.channels = 1;               // Set to 0 to use the device's native channel count.
    config.sampleRate        = std::ceil(mCPU->GetClockFrequency());           // Set to 0 to use the device's native sample rate.
    config.dataCallback      = data_callback;   // This function will be called when miniaudio needs more data.
    config.pUserData         = &mAudioBuffer;   // Can be accessed from the device object (device.pUserData).


    mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::ERROR, "Config {0}\n", config.periodSizeInFrames);

    ma_result result = ma_device_init(&*mAudioContext, &config, &*mAudioDevice);
    if (result != MA_SUCCESS) {
        mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::ERROR, "Failed to initialize miniaudio device: {0}\n", static_cast<uint8_t>(result));
    }
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
        float Radian = std::fmod(double(mCPU->GetCycleCount() - CPUCycles + i), mCPU->GetClockFrequency());
        Radian /= mCPU->GetClockFrequency();

        /*
         float Pulse1 = (sin(Radian* * 6.283185307f) + 1) * 7;

         float NextSample = DACOutput(Pulse1, Pulse1, Pulse1, Pulse1, Pulse1*2);
         NextSample = NextSample*2.f - 1.f;
         */

        float Pulse1 = sin(Radian * 6.283185307f * (200 + 25*cos(6.283185307f*Radian)));
        mAudioBuffer.Push(Pulse1);
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
        if (!bStartedDevice)
        {
            mLog->Log(ELOGGING_SOURCES::APU, ELOGGING_MODE::INFO, "Started Audio Device\n");
            ma_device_start(&*mAudioDevice);
            bStartedDevice = true;
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

float APU::DACOutput(uint8_t Pulse1, uint8_t Pulse2, uint8_t Triangle, uint8_t Noise, uint8_t DMC)
{
    // TODO: Some sort of mixer probably
    float PulseOut = SquareLUT[Pulse1 + Pulse2];

    float TNDOut = TNDLUT[3*Triangle + 2*Noise + DMC];

    return PulseOut + TNDOut;
}
