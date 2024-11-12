#ifndef TIMER_H
#define TIMER_H

#include <chrono>


class SimpleTimer {
    using Clock = std::chrono::high_resolution_clock;
public:
    SimpleTimer()
        : LastTime(Clock::now())
    {}

    inline void Restart() {
        LastTime = Clock::now();
    }

    inline unsigned int GetElapsedTimeAsMs() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - LastTime).count();
    }

private:
    Clock::time_point LastTime;
};


#endif // TIMER_H
