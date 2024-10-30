#include "timer.h"


Timer::Timer()
    : LastTime(Clock::now())
{}

void Timer::Restart() {
    LastTime = Clock::now();
}

unsigned int Timer::GetElapsedTimeAsMs() const {
    auto now = Clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - LastTime).count();
}
