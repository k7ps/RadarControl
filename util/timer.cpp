#include "timer.h"


SimpleTimer::SimpleTimer()
    : LastTime(Clock::now())
{}

void SimpleTimer::Restart() {
    LastTime = Clock::now();
}

unsigned int SimpleTimer::GetElapsedTimeAsMs() const {
    auto now = Clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - LastTime).count();
}
