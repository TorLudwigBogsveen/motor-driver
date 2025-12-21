#ifndef CLOCK_HPP
#define CLOCK_HPP
#include <cstdint>

/// @brief A simple timer utility for measuring elapsed time
class Clock {
public:
    /// @brief Construct a new Clock object
    /// @note Clock is initialized in stopped state
    Clock();
    /// @brief Start the clock
    /// @note If clock is already running, this resets the start time
    void start();

    /// @brief Stop the clock
    /// @note Captures the end time for elapsed() calculation
    void stop();

    /// @brief Get the elapsed time since start() was called
    /// @return Elapsed time in milliseconds as a std::uint64_t
    /// @note Returns 0 if clock has not been started
    std::uint64_t elapsed_millis() const;

    /// @brief Get the elapsed time since start() was called
    /// @return Elapsed time in microseconds as a std::uint64_t
    /// @note Returns 0 if clock has not been started
    std::uint64_t elapsed_micros() const;

    /// @brief Busy-wait for a specified duration
    /// @param milliseconds Duration to wait in milliseconds
    void busyWait(std::uint64_t milliseconds) const;

private:
    std::uint64_t startTime;  ///< Timestamp when clock was started
    std::uint64_t endTime;    ///< Timestamp when clock was stopped
    bool running;             ///< Flag indicating if clock is currently running
};

#endif