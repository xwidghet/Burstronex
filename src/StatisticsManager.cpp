#include "StatisticsManager.h"

void StatisticsManager::UpdateAPUStatistics(const APUStatistics& APUStatistics)
{
    std::lock_guard<std::mutex> mLockGuard(mMutex);

    mStatistics.mAPUStatistics = APUStatistics;
}

void StatisticsManager::UpdateEmulationStatistics(const EmulationStatistics& EmulationStatistics)
{
    std::lock_guard<std::mutex> mLockGuard(mMutex);

    mStatistics.mEmulationStatistics = EmulationStatistics;
}

StatisticsManager::Statistics StatisticsManager::GetStatistics()
{
    std::lock_guard<std::mutex> mLockGuard(mMutex);

    return mStatistics;
}
