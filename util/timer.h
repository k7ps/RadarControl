#ifndef TIMER_H
#define TIMER_H

#include <chrono>


class SimpleTimer {
    using Clock = std::chrono::high_resolution_clock;
public:
    SimpleTimer();

    void Restart();
    unsigned int GetElapsedTimeAsMs() const;

private:
    Clock::time_point LastTime;
};


#endif // TIMER_H
