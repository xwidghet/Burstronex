#pragma once

#include "CircularBuffer.h"
#include "StatisticsManager.h"

#include <cstdint>
#include <memory>

class MemoryMapper;
class CPU;
enum class ECPU_TIMING;

class ma_device;

static const uint32_t TARGET_SAMPLE_RATE = 48000;

static const uint16_t PULSE1_ENVELOPE_ADDRESS = 0x4000;
static const uint16_t PULSE1_SWEEP_ADDRESS = 0x4001;
static const uint16_t PULSE1_TIMER_ADDRESS = 0x4002;
static const uint16_t PULSE1_LENGTHCOUNTER_ADDRESS = 0x4003;

static const uint16_t PULSE2_ENVELOPE_ADDRESS = 0x4004;
static const uint16_t PULSE2_SWEEP_ADDRESS = 0x4005;
static const uint16_t PULSE2_TIMER_ADDRESS = 0x4006;
static const uint16_t PULSE2_LENGTHCOUNTER_ADDRESS = 0x4007;

static const uint16_t TRIANGLE_LINEARCOUNTER_ADDRESS = 0x4008;

// Address at 0x400A because 0x4009 is unused
static const uint16_t TRIANGLE_TIMER_ADDRESS = 0x400A;
static const uint16_t TRIANGLE_LENGTHCOUNTER_ADDRESS = 0x400B;

static const uint16_t NOISE_ENVELOPE_ADDRESS = 0x400C;
static const uint16_t NOISE_UNUSED = 0x400D;
static const uint16_t NOISE_MODE_PERIOD_ADDRESS = 0x400E;
static const uint16_t NOISE_LENGTHCOUNTER_ADDRESS = 0x400F;

static const uint16_t DMC_TIMER_ADDRESS = 0x4010;
static const uint16_t DMC_MEMORYREADER_ADDRESS= 0x4011;
static const uint16_t DMC_SAMPLEBUFFER_ADDRESS = 0x4012;
static const uint16_t DMC_OUTPUTUNIT_ADDRESS = 0x4013;

static const uint16_t STATUS_ADDRESS = 0x4015;
static const uint16_t FRAMECOUNTER_ADDRESS = 0x4017;

static const uint32_t NTSC_FRAME_INTERRUPT_CYCLE_COUNT = 29830;
static const uint32_t PAL_FRAME_INTERRUPT_CYCLE_COUNT = 33254;

static const uint32_t NTSC_SEQUENCE_STEP_CYCLE_COUNT = 3728;
static const uint32_t PAL_SEQUENCE_STEP_CYCLE_COUNT = 4156;

enum class EPULSE_ENVELOPE_MASKS : uint8_t {
    // D, Width of the pulse
    DUTY = 0b11000000,
    // L, 1 = Infinite, 0 = One Shot. If 1, Length Counter is frozen and envelope repeats forever.
    //    Length Counter and Envelope are clocked by Frame Counter (240hz).
    //    If Length counter is 0 this channel is silenced regardless of this bits value.
    //    When looping, Volume restarts at 15.
    ENVELOPE_LOOP = 0b00100000,
    // C. 1 = Constant, 0 = Use Envelope, with Volume starting at 15 and lowering over time.
    CONSTANT_VOLUME = 0b00010000,
    // V, Sets Volume if Constant, or rate of decay of Volume.
    VOLUME_ENVELOPE = 0b00001111
};

enum class EPULSE_SWEEP_MASKS : uint8_t {
    // E
    SWEEP_UNIT_ENABLED = 0b10000000,
    // P
    SWEEP_UNIT_PERIOD = 0b01110000,
    // N
    SWEEP_UNIT_NEGATE = 0b00001000,
    // S
    SWEEP_UNIT_SHIFT = 0b00000111
};

enum class EPULSE_TIMER_MASKS : uint8_t {
    // T
    TIMER_LOW = 0b11111111
};

// When written, reloads length counter, restarts envelope, resets phase of pulse generator.
enum class EPULSE_LENGTH_COUNTER_MASKS : uint8_t {
    // L
    LENGTH_COUNTER_LOAD = 0b11111000,
    // T
    TIMER_HIGH = 0b00000111
};

enum class ETRIANGLE_LINEAR_COUNTER_MASKS : uint8_t {
    // C, Controls Length Counter and Linear Counter at the same time.
    //    When set, halts the Length Counter,
    //    When set, prevents Linear Counter's internal reload flag from clearing, which halts it if TRIANGLE_LINEARCOUNTER_ADDRESS is written.
    //    Linear Counter silences the channel after a specified time.
    //    Since both Length Counter and Linear Counter are used, whichever is shorter wins.
    LENGTH_COUNTER_HALT_LINEAR_COUNTER_CONTROL = 0b10000000,
    // R, Applies to the Linear Counter on the next Frame Counter Tick, but only if the Reload Flag is set.
    //    A write to TRIANGLE_LINEARCOUNTER_ADDRESS is required to raise the reload flag.
    //    After a Frame Counter Tick, Appies the Load Value R, and then only clears the reload flag if C is also clear, otherwise continuous reload (halt)
    LINEAR_COUNTER_LOAD = 0b01111111
};

enum class ETRIANGLE_TIMER_MASKS : uint8_t {
    // T
    TIMER_LOW = 0b11111111
};

enum class ETRIANGLE_LENGTHCOUNTER_MASKS : uint8_t {
    // L
    LENGTH_COUNTER_LOAD = 0b11111000,
    // T, Timer High, and Linear Counter Reload somehow?
    TIMER_HIGH = 0b00000111
};

enum class ENOISE_ENVELOPE_MASKS : uint8_t {
    // L
    ENVELOPE_LOOP_LENGTH_COUNTER_HALT = 0b00100000,
    // C
    CONSTANT_VOLUME = 0b00010000,
    // V
    VOLUME_ENVELOPE = 0b00001111
};

enum class ENOISE_MODE_PERIOD_MASKS : uint8_t {
    // M, When set, period is drastically shortened
    //    Buzzing/Metalic tone, depending on what bits happen to be in the generator when switched to periodic.
    NOISE_MODE = 0b10000000,
    // P, Determines the frequency of noise by loading a period from a lookup table.
    NOISE_PERIOD = 0b00001111
};

enum class ENOISE_LENGTHCOUNTER_MASKS : uint8_t {
    // L
    LENGTH_COUNTER_LOAD = 0b11111000,
};

// DMC_TIMER_ADDRESS
enum class EDMC_TIMER_MASKS : uint8_t {
    // I
    IRQ_ENABLE = 0b10000000,
    // L
    LOOP = 0b01000000,
    // R
    FREQUENCY = 0b00001111
};

// DMC_MEMORYREADER_ADDRESS
enum class EDMC_MEMORYREADER_MASKS : uint8_t {
    // D
    LOAD_COUNTER = 0b01111111
};

// DMC_SAMPLEBUFFER_ADDRESS
enum class EDMC_SAMPLEBUFFER_MASKS : uint8_t {
    // A
    ADDRESS = 0b11111111
};

// DMC_OUTPUTUNIT_ADDRESS
enum class EDMC_OUTPUTUNIT_MASKS : uint8_t {
    // L
    LENGTH = 0b11111111
};

// STATUS_ADDRESS
// Seems like there could be differences between read/write, but I think if I understand it, it's the same it's just both the APU and CPU can write.
// Ex. NOISE_PLAYING == 0 if CPU set 0, or if CPU set 1 and then length counter hit 0.
//
// Reading this register clears the FRAME_INTERRUPT flag
// Bit 5 is open bus, values comes from the last cycle that did not read STATUS_ADDRESS.
enum class ESTATUS_READ_MASKS : uint8_t {
    // I
    DMC_INTERRUPT = 0b10000000,
    // F
    FRAME_INTERRUPT = 0b01000000,
    // D, 1 if DMC bytes remaining > 0
    DMC_ACTIVE = 0b00010000,
    // N, 1 if length counter > 0, 0 if 0 was written here or length counter == 0 for Noise
    NOISE_PLAYING = 0b00001000,
    // T, 1 if length counter > 0, 0 if 0 was written here or length counter == 0  for Triangle. Linear counter is ignored.
    TRIANGLE_PLAYING = 0b00001000,
    // 2, 1 if length counter > 0, 0 if 0 was written here or length counter == 0  for Pulse 2
    PULSE_2_PLAYING = 0b00000010,
    // 1,  1 if length counter > 0, 0 if 0 was written here or length counter == 0  for Pulse 1
    PULSE_1_PLAYING = 0b00000001,
};

// STATUS_ADDRESS
enum class ESTATUS_WRITE_MASKS : uint8_t {
    // D, When 0, set remaining bytes to 0 then silence DMC
    //    When 1, DMC Sample will be restarted only if bits remaining is 0.
    //            If bits remaining, these will finish playing before next sample plays.
    DMC_ACTIVE = 0b00010000,
    // N, When 0, silence channel and halt length counter
    NOISE_PLAYING = 0b00001000,
    // T, When 0, silence channel and halt length counter
    TRIANGLE_PLAYING = 0b00001000,
    // 2, When 0, silence channel and halt length counter
    PULSE_2_PLAYING = 0b00000010,
    // 1, When 0, silence channel and halt length counter
    PULSE_1_PLAYING = 0b00000001,
};

// FRAMECOUNTER_ADDRESS
enum class EFRAME_COUNTER_MASKS : uint8_t {
    // M
    // (Copy paste from NESDEV)
    // mode 0:    mode 1:       function
    // ---------  -----------  -----------------------------
    // - - - f    - - - - -    IRQ (if bit 6 is clear)
    // - l - l    - l - - l    Length counter and sweep
    // e e e e    e e e - e    Envelope and linear counter
    //
    // Frame counter ticks ~4 times per frame (240hz NTSC)
    //
    // The above are quater and fifth frame steps for each mode.
    // Takes affect 2-3 clocks afer write, depending on if the next clock is odd or even.
    // When MODE is set, all units reset to the begining of the 5 step mode.
    // When clear, only the sequence is reset without clocking units.
    //
    // Some games synchronize the APU by enabling this bit once per frame.
    MODE = 0b10000000,
    // I, blocks mode 0 IRQ.
    IRQ_INHIBIT = 0b01000000
};

static const std::array<uint8_t, 4> DUTY_CYCLE_SEQUENCES {
    0b01000000,
    0b01100000,
    0b01111000,
    0b10011111
};

static const std::array<uint8_t, 32> LENGTH_COUNTER_TABLE {
    10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14,
    12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
};


static const std::array<uint8_t, 32> TRIANGLE_SEQUENCE_TABLE {
    15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
};

class APU {
    MemoryMapper* mRAM;
    CPU* mCPU;

    struct ApuRegisters {
        uint8_t Pulse1_Timer;
        uint8_t Pulse1_LengthCounter;
        uint8_t Pulse1_Envelope;
        uint8_t Pulse1_Sweep;

        uint8_t Pulse2_Timer;
        uint8_t Pulse2_LengthCounter;
        uint8_t Pulse2_Envelope;
        uint8_t Pulse2_Sweep;

        uint8_t Triangle_Timer;
        uint8_t Triangle_LengthCounter;
        uint8_t Triangle_LinearCounter;

        uint8_t Noise_Envelope;
        uint8_t Noise_ModePeriod;
        uint8_t Noise_LengthCounter;

        uint8_t DMC_Timer;
        uint8_t DMC_MemoryReader;
        uint8_t DMC_SampleBuffer;
        uint8_t DMC_OutputUnit;

        uint8_t Status;
        uint8_t FrameCounter;
    } mRegisters;

    // Simple solution for tracking leftover cycles from running at half the speed of the CPU.
    // I wouldn't need to do this if the CPU ran one cycle at a time, since I could just % 2 the cycle count, but I don't do that currently.
    // Ex.
    //      Leftovers: 7 + 7 = 14, 14 / 2 = 7 APU cycles
    //      Dividing: 7/2 = 3, 3 * 2 = 6 APU cycles, missing one cycle after running two 7 cycle instructions.
    int32_t mCyclesToRun = 0;

    uint32_t mCyclesSinceFrameInterrupt = 0;
    uint32_t mFrameInterruptCycleCount = 0;
    uint32_t mSequenceCycleCount = 0;
    uint32_t mSequenceCycleInterval = 0;

    uint64_t mAudioFrameCount = 0;

    uint8_t mSequenceIndex = 0;

    std::unique_ptr<ma_device> mAudioDevice;
    bool mbStartedDevice = false;

    // Circular buffer of two audio frames. Ideally it stays roughly 1 frame full.
    CircularBuffer<float, NTSC_FRAME_INTERRUPT_CYCLE_COUNT*3> mAudioBuffer;

    APUStatistics mAPUStatistics;

    struct EnvelopeUnit {
        bool mbReloadFlag = false;
        uint8_t mDivider = 0;

        uint8_t mValue = 15;
    };

    struct SweepUnit {
        bool mbIsEnabled = false;
        bool mbIsMutingChannel = false;

        uint8_t mDivider = 0;
        bool mbReloadFlag = false;

        uint16_t mPeriod = 0;
        uint16_t mPeriodLength = 0;
        uint16_t mTargetPeriod = 0;
    };

    struct PulseUnit {
        EnvelopeUnit mEnvelope;
        SweepUnit mSweep;
        uint8_t mSequencerIndex = 0;
        uint8_t mLengthCounter = 0;

        bool mbIsPulse2 = false;

        uint8_t mOutputSample = 0;

        void ClockSequencer(const uint8_t TimerRegister, const uint8_t LengthCounterRegister);
        void ClockEnvelope(const uint8_t EnvelopeRegister);
        void ClockHalfFrameSweep(const uint8_t SweepRegister);
        void ClockSweep(const uint8_t SweepRegister);
        void ClockLengthCounter(bool bInfinite);
        void Execute(const uint8_t EnvelopeRegister);
    };

    struct TriangleUnit {
        uint16_t mTimer = 0;
        uint8_t mLengthCounter = 0;
        uint8_t mLinearCounter = 0;
        bool mbLinearCounterReloadFlag = false;
        uint8_t mSequenceIndex = 0;

        bool mbIsEnabled = false;
        uint8_t mOutputSample = 0;

        void ClockTimer(const uint8_t TimerRegister, const uint8_t LengthCounterRegister);
        void ClockLengthCounter(const uint8_t LinearCounterRegister);
        void ClockLinearCounter(const uint8_t LinearCounterRegister);
        void ClockSequencer();
    };

    struct NoiseUnit {
        uint8_t mTimer = 0;
        EnvelopeUnit mEnvelope;
        uint8_t mLengthCounter = 0;
        uint16_t mLFSR = 1;

        std::array<uint16_t, 16> mNoiseTimerLUT;

        bool mbIsEnabled = false;
        uint8_t mOutputSample = 0;

        void ClockTimer(const uint8_t ModePeriodRegister);
        void ClockEnvelope(const uint8_t EnvelopeRegister);
        void ClockLengthCounter(const uint8_t EnvelopeRegister);
        void ClockSequencer(const uint8_t ModePeriodRegister);
    };

    PulseUnit mPulse1;
    PulseUnit mPulse2;

    TriangleUnit mTriangle;

    NoiseUnit mNoise;

public:
    APU();
    ~APU();

    void Init(MemoryMapper* MemoryMapper, CPU* CPU, const ECPU_TIMING Timing);

    void Execute(const uint8_t CPUCycles);

    float GetBufferFillPercentage() const;

    void WritePulse1_Timer(const uint8_t Data);
    void WritePulse1_LengthCounter(const uint8_t Data);
    void WritePulse1_Envelope(const uint8_t Data);
    void WritePulse1_Sweep(const uint8_t Data);

    void WritePulse2_Timer(const uint8_t Data);
    void WritePulse2_LengthCounter(const uint8_t Data);
    void WritePulse2_Envelope(const uint8_t Data);
    void WritePulse2_Sweep(const uint8_t Data);

    void WriteTriangle_Timer(const uint8_t Data);
    void WriteTriangle_LengthCounter(const uint8_t Data);
    void WriteTriangle_LinearCounter(const uint8_t Data);

    void WriteNoise_Envelope(const uint8_t Data);
    void WriteNoise_LengthCounter(const uint8_t Data);
    void WriteNoise_ModePeriod(const uint8_t Data);

    void WriteDMC_Timer(const uint8_t Data);
    void WriteDMC_MemoryReader(const uint8_t Data);
    void WriteDMC_SampleBuffer(const uint8_t Data);
    void WriteDMC_OutputUnit(const uint8_t Data);

    uint8_t ReadStatus();
    void WriteStatus(const uint8_t Data);

    void WriteFrameCounter(const uint8_t Data);

private:
    void InitAudio();

    void ExecuteCycle();

    void ExecuteSequencer();

    void ExecuteMode0Sequencer();

    void ExecuteMode1Sequencer();

    void HalfFrame();

    void QuarterFrame();

    float DACOutput();

    float TestOutput(uint8_t CPUCycles, uint8_t i);
};
