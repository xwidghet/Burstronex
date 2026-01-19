#pragma once

#include <cstdint>
#include <iostream>
#include <format>
#include <memory>

enum class ELOGGING_MODE {
    VERBOSE,
    INFO,
    WARNING,
    ERROR,
    NONE
};

enum class ELOGGING_SOURCES {
    CPU = 1 << 0,
    APU = 1 << 1,
    PPU = 1 << 2,
    MEMORY_MAPPER = 1 << 3,
    ROM_LOADER = 1 << 4,
    GNES = 1 << 5,
    RENDERER = 1 << 6,
    ALL = 0xFFFF
};

class Logger {
    ELOGGING_MODE mLoggingMode = ELOGGING_MODE::INFO;
    uint32_t mLoggingSources = 0xFFFF;

public:
    template<typename... Args>
    void Log(ELOGGING_SOURCES LogSource, ELOGGING_MODE LogMode,std::format_string<Args...> Format, Args&&... Arguments)
    {
        if ((mLoggingSources & static_cast<uint8_t>(LogSource)) && (LogMode >= mLoggingMode))
        {
            std::cout << std::format(Format, std::forward<Args>(Arguments)...);
        }
    }

    void SetLogSource(ELOGGING_SOURCES LogSource, bool bEnabled);
};

inline std::unique_ptr<Logger> mLog;
