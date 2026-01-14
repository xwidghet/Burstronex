#pragma once

#include <chrono>

class Timer {
    std::chrono::time_point<std::chrono::high_resolution_clock> StartTime;

public:
    Timer();

    void Reset();

    double PeakDeltaTime() const;

    double GetDeltaTime();

    // Waits until the time between the timer's start time and now becomes the argument
    void WaitUntil(double Seconds);
};
