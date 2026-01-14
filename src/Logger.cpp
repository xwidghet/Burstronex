#include "Logger.h"

void Logger::SetLogSource(ELOGGING_SOURCES LogSource, bool bEnabled)
{
    if (bEnabled)
        mLoggingSources |= static_cast<uint32_t>(LogSource);
    else
        mLoggingSources &= ~static_cast<uint32_t>(LogSource);
}
