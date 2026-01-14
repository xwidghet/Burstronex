#pragma once

#include <cstdint>

class MemoryMapper;

static const uint16_t PULSE1_TIMER_ADDRESS = 0x4000;
static const uint16_t PULSE1_LENGTHCOUNTER_ADDRESS = 0x4001;
static const uint16_t PULSE1_ENVELOPE_ADDRESS = 0x4002;
static const uint16_t PULSE1_SWEEP_ADDRESS = 0x4003;

static const uint16_t PULSE2_TIMER_ADDRESS = 0x4004;
static const uint16_t PULSE2_LENGTHCOUNTER_ADDRESS = 0x4005;
static const uint16_t PULSE2_ENVELOPE_ADDRESS = 0x4006;
static const uint16_t PULSE2_SWEEP_ADDRESS = 0x4007;

static const uint16_t TRIANGLE_TIMER_ADDRESS = 0x4008;

// Address at 0x400A because 0x4009 is unused
static const uint16_t TRIANGLE_LENGTHCOUNTER_ADDRESS = 0x400A;
static const uint16_t TRIANGLE_LINEARCOUNTER_ADDRESS = 0x400B;

static const uint16_t NOISE_TIMER_ADDRESS = 0x400C;
static const uint16_t NOISE_LENGTHCOUNTER_ADDRESS = 0x400D;
static const uint16_t NOISE_ENVELOPE_ADDRESS = 0x400E;
static const uint16_t NOISE_LINEARFEEDBACKSHIFTREGISTER_ADDRESS = 0x400F;

static const uint16_t DMC_TIMER_ADDRESS = 0x4010;
static const uint16_t DMC_MEMORYREADER_ADDRESS= 0x4011;
static const uint16_t DMC_SAMPLEBUFFER_ADDRESS = 0x4012;
static const uint16_t DMC_OUTPUTUNIT_ADDRESS = 0x4013;

static const uint16_t CHANNELENABLE_LENGTHCOUNTERSTATUS_ADDRESS = 0x4015;
static const uint16_t FRAMECOUNTER_ADDRESS = 0x4017;

class APU {
    MemoryMapper* mRAM;

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

        uint8_t Noise_Timer;
        uint8_t Noise_LengthCounter;
        uint8_t Noise_Envelope;
        uint8_t Noise_LinearFeedbackShiftRegister;

        uint8_t DMC_Timer;
        uint8_t DMC_MemoryReader;
        uint8_t DMC_SampleBuffer;
        uint8_t DMC_OutputUnit;

        uint8_t ChannelEnable_LengthCounterStatus;
        uint8_t FrameCounter;
    } mRegisters;

    // Simple solution for tracking leftover cycles from running at half the speed of the CPU.
    // I wouldn't need to do this if the CPU ran one cycle at a time, since I could just % 2 the cycle count, but I don't do that currently.
    // Ex.
    //      Leftovers: 7 + 7 = 14, 14 / 2 = 7 APU cycles
    //      Dividing: 7/2 = 3, 3 * 2 = 6 APU cycles, missing one cycle after running two 7 cycle instructions.
    int32_t mCyclesToRun = 0;

public:
    void Init(MemoryMapper* MemoryMapper);

    void Execute(const uint8_t CPUCycles);

private:
    void UpdateRegisters();

    void ExecuteCycle();
};
