//# Copyright (c) 2025 MDU Solar Team
#ifndef ERROR_HPP
#define ERROR_HPP

#include <stdint.h>

typedef uint16_t ControllerResult;

/*
  Error Codes
*/

#define NUM_ERRORS 16

#define E_SUCCESS 0
#define E_MOTOR_OVER_SPEED (1 << 0)               // Motor Over Speed (15% overshoot above max RPM)
#define E_DESATURATION_FAULT (1 << 1)             // Desaturation Fault (IGBT desaturation, IGBT driver OVLO)
#define E_15V_RAIL_UNDER_VOLTAGE (1 << 2)         // 15V Rail under voltage lock out (UVLO)
#define E_CONFIG_READ_ERROR (1 << 3)              // Config read error (some values may be reset to defaults)
#define E_WATCHDOG_CAUSED_LAST_RESET (1 << 4)     // Watchdog caused last reset
#define E_BAD_MOTOR_POSITION (1 << 5)             // Bad motor position hall sequence
#define E_DC_BUS_OVERVOLTAGE (1 << 6)             // DC Bus over voltage
#define E_SOFTWARE_OVERCURRENT (1 << 7)           // Software over current
#define E_HARDWARE_OVERCURRENT (1 << 8)           // Hardware over current
#define E_NOT_STATIONARY_BIT (1 << 9)
#define E_CANNOT_SET_MODE_BIT (1 << 10)
#define E_WRONG_DRIVE_MODE_BIT (1 << 11)
#define E_NEGATIVE_FLOAT_BIT (1 << 12)
#define E_PERCENTAGE_OUT_OF_RANGE_BIT (1 << 13)
#define E_OVERHEAT (1 << 14)
#define E_CAN (1 << 15)

/*
  Motor Limits
*/

#define NUM_LIMITS 7

#define L_OUTPUT_VOLTAGE_PWM (1 << 0)
#define L_MOTOR_CURRENT (1 << 1)
#define L_VELOCITY (1 << 2)
#define L_BUS_CURRENT (1 << 3)
#define L_BUS_VOLTAGE_UPPER_LIMIT (1 << 4)
#define L_BUS_VOLTAGE_LOWER_LIMIT (1 << 5)
#define L_IPM_OR_MOTOR_TEMPRATURE (1 << 6)

#endif