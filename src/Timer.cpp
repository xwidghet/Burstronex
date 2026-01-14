#include "Timer.h"

Timer::Timer()
{
    StartTime = std::chrono::steady_clock::now();
}

void Timer::Reset()
{
    StartTime = std::chrono::steady_clock::now();
}

double Timer::PeakDeltaTime() const
{
    auto TimePassed = std::chrono::steady_clock::now() - StartTime;
    return std::chrono::duration<double>(TimePassed).count();
}

double Timer::GetDeltaTime()
{
    auto TimePassed = std::chrono::steady_clock::now() - StartTime;
    double DeltaTime = std::chrono::duration<double>(TimePassed).count();

    StartTime = std::chrono::steady_clock::now();

    return DeltaTime;
}
