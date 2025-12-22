//# Copyright (c) 2025 MDU Solar Team
#ifndef ERROR_HPP
#define ERROR_HPP

#include <cstdint>

enum class MotorFlags : std::uint32_t {
  Success = 0,
  HardwareOvercurrent = (1 << 0),          // Hardware over current
  SoftwareOvercurrent = (1 << 1),          // Software over current
  DcBusOvervoltage = (1 << 2),             // DC Bus over voltage
  BadMotorPosition = (1 << 3),             // Bad motor position hall sequence
  WatchdogCausedLastReset = (1 << 4),      // Watchdog caused last reset
  ConfigReadError = (1 << 5),              // Config read error (some values may be reset to defaults)
  Rail15vUnderVoltage = (1 << 6),          // 15V Rail under voltage lock out (UVLO)
  DesaturationFault = (1 << 7),            // Desaturation Fault (IGBT desaturation, IGBT driver OVLO)
  MotorOverSpeed = (1 << 8),               // Motor Over Speed (15% overshoot above max RPM)
  NotStationary = (1 << 9),                // Motor not stationary
  CannotSetMode = (1 << 10),               // Cannot set drive mode
  WrongDriveMode = (1 << 11),              // Drive mode not supported
  NegativeFloat = (1 << 12),               // Negative float value where not allowed
  PercentageOutOfRange = (1 << 13),        // Percentage value out of range
  Overheat = (1 << 14),                    // Overheat fault
  Can = (1 << 15),                         // CAN communication fault
  Timeout = (1 << 16),                     // Timeout occurred
};

inline MotorFlags operator|(MotorFlags a, MotorFlags b) {
    return static_cast<MotorFlags>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b)
    );
}

inline MotorFlags operator&(MotorFlags a, MotorFlags b) {
    return static_cast<MotorFlags>(
        static_cast<uint32_t>(a) & static_cast<uint32_t>(b)
    );
}

inline MotorFlags& operator|=(MotorFlags& a, MotorFlags b) {
    a = a | b;
    return a;
}

inline bool any(MotorFlags f) {
    return static_cast<uint32_t>(f) != 0;
}

enum class MotorLimit : std::uint32_t {
  OutputVoltagePwm = (1 << 0),
  MotorCurrent = (1 << 1),
  Velocity = (1 << 2),
  BusCurrent = (1 << 3),
  BusVoltageUpperLimit = (1 << 4),
  BusVoltageLowerLimit = (1 << 5),
  IpmOrMotorTemperature = (1 << 6)
};

#endif
