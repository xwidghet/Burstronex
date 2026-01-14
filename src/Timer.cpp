#include "Timer.h"

#include <xmmintrin.h>

Timer::Timer()
{
    StartTime = std::chrono::high_resolution_clock::now();
}

void Timer::Reset()
{
    StartTime = std::chrono::high_resolution_clock::now();
}

double Timer::PeakDeltaTime() const
{
    auto TimePassed = std::chrono::high_resolution_clock::now() - StartTime;
    return std::chrono::duration<double>(TimePassed).count();
}

double Timer::GetDeltaTime()
{
    auto TimePassed = std::chrono::high_resolution_clock::now() - StartTime;
    double DeltaTime = std::chrono::duration<double>(TimePassed).count();

    StartTime = std::chrono::high_resolution_clock::now();

    return DeltaTime;
}

void Timer::WaitUntil(double Seconds)
{
    std::chrono::duration<double> WaitDuration(Seconds);

    auto EndTime = StartTime + WaitDuration;

    while (std::chrono::high_resolution_clock::now() < EndTime)
    {
        _mm_pause();
    }
}
