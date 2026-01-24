#pragma once

#include <memory>
#include <mutex>

struct EmulationStatistics {
    float mEmulationSpeed = 0.f;
};

struct APUStatistics {
    float mBufferFillPercentage = 0.f;
};

class StatisticsManager {
    struct Statistics {
        EmulationStatistics mEmulationStatistics;
        APUStatistics mAPUStatistics;
    } mStatistics;

    std::mutex mMutex;

public:
    // Thread Safe
    void UpdateAPUStatistics(const APUStatistics& APUStatistics);

    // Thread Safe
    void UpdateEmulationStatistics(const EmulationStatistics& EmulationStatistics);

    // Intentionally copies to avoid needing a thread safe container returned.
    // If the statistics amount increases to a degree that thread safety would be faster, then I'll rework this.
    // Thread Safe
    Statistics GetStatistics();
};

inline std::unique_ptr<StatisticsManager> mStatisticsManager;
