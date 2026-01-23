// # Copyright (c) 2025 MDU Solar Team
#ifndef ERROR_HPP
#define ERROR_HPP

#include <cstdint>

enum class MotorFlags : std::uint32_t
{
    Success = 0,
    ControllerError = (1 << 0),      // Motor controller reported an error
    NotStationary = (1 << 1),        // Motor not stationary
    CannotSetMode = (1 << 2),        // Cannot set drive mode
    WrongDriveMode = (1 << 3),       // Drive mode not supported
    NegativeFloat = (1 << 4),        // Negative float value where not allowed
    PercentageOutOfRange = (1 << 5), // Percentage value out of range
    Can = (1 << 7),                  // CAN communication fault
    Timeout = (1 << 8),              // Timeout occurred
};

inline MotorFlags operator|(MotorFlags a, MotorFlags b)
{
    return static_cast<MotorFlags>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline MotorFlags operator&(MotorFlags a, MotorFlags b)
{
    return static_cast<MotorFlags>(
        static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline MotorFlags &operator|=(MotorFlags &a, MotorFlags b)
{
    a = a | b;
    return a;
}

inline bool any(MotorFlags f)
{
    return static_cast<uint32_t>(f) != 0;
}

enum class MotorLimit : std::uint32_t
{
    OutputVoltagePwm = (1 << 0),
    MotorCurrent = (1 << 1),
    Velocity = (1 << 2),
    BusCurrent = (1 << 3),
    BusVoltageUpperLimit = (1 << 4),
    BusVoltageLowerLimit = (1 << 5),
    IpmOrMotorTemperature = (1 << 6)
};

#endif
