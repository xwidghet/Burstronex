#pragma once

#include <cstdint>
#include <string>
#include <vector>

enum class ECPU_TIMING
{
    NTSC,
    PAL,
    DENDY,
    PC,
    CPU_TIMING_MAX
};

enum class ENameTableLayout : uint8_t {
    Horizontal,
    Vertical,
    Alternative,
    NAME_TABLE_LAYOUT_MAX
};

// Vs. PPU types (Header byte 13 D3..D0)
enum class EVsSystemType : uint8_t {
    RP2C03_RP2C03,
    RP2C04_0001,
    RP2C04_0002,
    RP2C04_0003,
    RP2C04_0004,
    RESERVED_0,
    RESERVED_1,
    RC2C05_01,
    // ($2002 AND $3F =$3D)
    RC2C05_02,
    // ($2002 AND $1F =$1C)
    RC2C05_03,
    // ($2002 AND $1F =$1B)
    RC2C05_04,
    RESERVED_2,
    RESERVED_3,
    RESERVED_4,
    RESERVED_5,
    VS_SYSTEM_TYPE_MAX
};

enum class EVsHardwareType : uint8_t {
    Unisystem_Normal,
    Unisystem_RBIBaseball,
    Unisystem_TKOBoxing,
    Unisystem_SuperXevious,
    Unisystem_VsIceClimberJapan,
    DualSystem_Normal,
    DualSystem_RaidOnBungelingBay,
    VS_HARDWARE_TYPE_MAX
};

enum class EExtendedConsoleType : uint8_t {
    NES,
    Nintendo_Vs_System,
    Playchoice10,
    NES_DecimalMode,
    NES_EPSM_PlugThrough,
    // Red/Cyan STN pallete
    VR_Technology_VT01,
    VR_Technology_VT02,
    VR_Technology_VT03,
    VR_Technology_VT09,
    VR_Technology_VT32,
    VR_Technology_VT369,
    UMC_UM6578,
    Famicon_Network_System,
    RESERVED_0,
    RESERVED_1,
    RESERVED_2,
    EXTENDED_CONSOLE_TYPE_MAX
};

// Header byte 15 indicates that the ROM expects a specific set of devices accessible at CPU $4016/$4017.
// Only adding ones I need for now...I'll add the test if I feel like building out true 100% NES compatability.
enum class EDefaultExpansionDevice : uint8_t {
    Unspecified,
    NES_Controllers,
    NES_Four_Score,
    NES_Four_Players_Adapter,
    // (1P via $4016)
    VS_System0,
    // (1P via $4017)
    VS_System1,
    RESERVED_0,
    VS_Zapper,
    // ($4017)
    Zapper,
    Two_Zappers,
    Bandai_Hyper_Shot_Lightgun,
    Power_Pad_Side_A,
    Power_Pad_Side_B,
    DEFAULT_EXPANSION_DEVICE_MAX
};

struct ROMData {
    std::string Name;

    ECPU_TIMING CPUTimingMode;
    ENameTableLayout NameTableLayout;

    EVsSystemType VsSystemType;
    EVsHardwareType VsHardwareType;
    EExtendedConsoleType ExtendedConsoleType;

    EDefaultExpansionDevice DefaultExpansionDevice;

    int32_t MiscelaneousROMCount;

    std::vector<char> TrainerAreaMemory;
    std::vector<char> PrgRomMemory;
    std::vector<char> ChrRomMemory;
    std::vector<char> MiscelaneousRomMemory;

    std::vector<char>PlayChoiceInstRomMemory;

    // Needed to decode PlayChoiceInstRomMemory
    uint16_t Playchoice10PromData;

    // Needed to decode PlayChoiceInstRomMemory, usually 00,00,00,00,FF,FF,FF,FF,00,00,00,00,FF,FF,FF,FF
    uint16_t Playchoice10PromCounterOut;
};
