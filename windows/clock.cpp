#include "clock.hpp"

// Add clock implementation here

#include <chrono>

Clock::Clock() : startTime(0), endTime(0), running(false) {}

void Clock::start() {
    startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    running = true;
}

void Clock::stop() {
    endTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    running = false;
}

std::uint64_t Clock::elapsed_millis() const {
    if (!running && startTime == 0) {
        return 0;
    }
    
    std::uint64_t end = running ? 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count() : 
        endTime;
    
    return static_cast<std::uint64_t>(end - startTime);
}

std::uint64_t Clock::elapsed_micros() const {
    if (!running && startTime == 0) {
        return 0;
    }
    
    std::uint64_t end = running ? 
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count() : 
        endTime;
    
    return static_cast<std::uint64_t>(end - startTime);
}

void Clock::busyWait(std::uint64_t milliseconds) const {
    auto start = std::chrono::high_resolution_clock::now();
    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed >= milliseconds) {
            break;
        }
    }
}