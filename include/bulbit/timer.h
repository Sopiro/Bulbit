#pragma once

#include "common.h"

namespace bulbit
{

class Timer
{
    using clock = std::chrono::steady_clock;

public:
    Timer();

    double Mark();
    void Reset();

private:
    std::vector<clock::time_point> time_points;
};

inline Timer::Timer()
{
    Mark();
}

inline double Timer::Mark()
{
    clock::time_point t = clock::now();
    std::chrono::duration<double> dt = t - time_points.back();

    time_points.push_back(t);
    return dt.count();
}

inline void Timer::Reset()
{
    time_points.clear();
    time_points.push_back(clock::now());
}

} // namespace bulbit
