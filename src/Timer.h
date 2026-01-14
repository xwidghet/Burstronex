#pragma once

#include <chrono>

class Timer {
    std::chrono::time_point<std::chrono::steady_clock> StartTime;

public:
    Timer();

    void Reset();

    double PeakDeltaTime() const;

    double GetDeltaTime();
};
