#pragma once

#include <cstdint>

// On write,
// first bit == 1 Controller state for the current button is constantly loaded
// first bit == 0 loads next button state (ex. A -> B -> Left) for both controllers
static const uint16_t CONTROLLER_STROBE_ADDRESS = 0x4016;

// Returns the state of the current button of controller port 1
static const uint16_t CONTROLLER_1_READ_ADDRESS = 0x4016;

// Returns the state of the current button of controller port 2
// Same address as APU's Frame Counter, this occurs when read while Frame Counter occurs on write.
static const uint16_t CONTROLLER_2_READ_ADDRESS = 0x4017;

enum class EControllerReadMasks {
    PRIMARY_CONTROLLER_STATUS = 1 << 0,
    EXPANSION_CONTROLLER_STATUS = 1 << 1,
    MICROPHONE_STATUS = 1 << 2
};

enum class EControllerButtonMasks {
    A = 1 << 0,
    B = 1 << 1,
    SELECT = 1 << 2,
    START = 1 << 3,
    UP = 1 << 4,
    DOWN = 1 << 5,
    LEFT = 1 << 6,
    RIGHT = 1 << 7
};
