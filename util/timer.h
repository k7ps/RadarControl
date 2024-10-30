#ifndef TIMER_H
#define TIMER_H

#include <chrono>


class Timer {
    using Clock = std::chrono::high_resolution_clock;
public:
    Timer();

    void Restart();
    unsigned int GetElapsedTimeAsMs() const;

private:
    Clock::time_point LastTime;
};


#endif // TIMER_H
