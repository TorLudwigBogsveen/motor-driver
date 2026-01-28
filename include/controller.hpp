// # Copyright (c) 2025 MDU Solar Team
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <stdint.h>
#include <error.hpp>
#include <input.hpp>
#include <can/can_protocol.hpp>

/// @def STARTUP_DRIVE_MODE
/// @brief Drive mode selected when the controller enters Startup state
#define STARTUP_DRIVE_MODE DriveMode::Current

/// @brief Motor rotation direction
/// @details Determines the sign applied to velocity and current commands
enum class MotorDirection : int8_t
{
    Forward = 1,   ///< Forward rotation
    Neutral = 0,   ///< No drive torque applied
    Backward = -1  ///< Reverse rotation
};

/// @brief High-level controller state
/// @details Represents the current operational state of the controller state machine
enum class ControllerState : int8_t
{
    Startup,  ///< Initial power-on / reset state
    Running,  ///< Normal driving operation
    Parking,  ///< Stationary / disabled drive state
    Error     ///< Error-latched state requiring intervention
};

/// @brief Drive control strategy
/// @details Determines how user inputs are translated into motor commands
enum class DriveMode : int8_t
{
    Current,   ///< Direct current (torque) control
    Velocity,  ///< Velocity closed-loop control
    Cruise,    ///< Cruise control (maintains target speed)
    Custom1    ///< Reserved for future or experimental modes
};

/// @brief Unified motor command output
/// @details Represents the desired motor current and velocity setpoints
struct SpeedCommand
{
    float current;   ///< Target motor current (percentage or absolute depending on mode)
    float velocity;  ///< Target motor velocity (RPM or m/s depending on system)
};

/// @brief Slider input abstraction used by the controller
/// @details Can be swapped between smoothed or raw slider implementations
using ControllerSliders = SmoothSliders;
// using ControllerSliders = Sliders;

/// @brief Main controller class for motor control logic
/// @details
/// The Controller class encapsulates:
/// - High-level state machine (startup, running, parking, error)
/// - Drive mode logic (current, velocity, cruise, custom)
/// - Input handling (buttons, sliders)
/// - Safety interlocks and limits
/// - Translation of user intent into motor commands
///
/// This class is **platform-agnostic** and communicates exclusively via
/// CAN frames and abstract input types.
///
/// Typical usage pattern:
/// @code
/// Controller controller;
/// controller.update(millis, micros);
/// controller.processIncomingCommand(frame);
/// controller.sendPeriodicMessages(current_time_ms, t_state, transmit_func);
/// @endcode
class Controller
{
public:
    /// @brief Construct a new Controller instance
    /// @details Initializes internal state to Startup and default drive mode
    Controller();

    /// @brief Main periodic update function
    /// @param millis Current system time in milliseconds
    /// @param micros Current system time in microseconds
    ///
    /// @details
    /// Must be called regularly (e.g., in a main loop).
    /// Handles state transitions, input processing, and internal timing.
    void update(uint32_t millis, uint32_t micros);

    /// @brief Process an incoming CAN frame
    /// @param frame Received CAN frame
    ///
    /// @details
    /// Updates internal telemetry and status based on motor controller feedback.
    void processIncomingCommand(const CanFrame &frame);

    /// @brief Send periodic CAN messages
    /// @param current_time_ms Current system time in milliseconds
    /// @param t_state User-provided context pointer
    /// @param transmit_func Callback used to transmit CAN frames
    ///
    /// @details
    /// This abstraction allows the Controller to remain independent of
    /// the underlying CAN driver implementation.
    void sendPeriodicMessages(
        uint32_t current_time_ms,
        void* t_state,
        void(*transmit_func)(const CanFrame&, void*)
    );

    /// @brief Check if the vehicle is stationary
    /// @param margin Allowed velocity margin
    /// @return true if velocity magnitude is below margin
    bool isStationary(float margin) const;

    /// @brief Check if controller is in an active drive mode
    bool isInDriveMode() const;

    /// @brief Check if accelerometer input is disabled
    bool isAccelerometerOff() const;

    /// @brief Toggle motor direction (Forward ↔ Backward)
    MotorFlags toggleDirection();

    /// @brief Set motor direction explicitly
    MotorFlags setDirection(MotorDirection direction);

    /// @brief Get current motor direction
    MotorDirection getDirection() const;

    /// @brief Set motor controller connection status
    /// @details Used to gate command transmission and error handling
    MotorFlags setMotorControllerConnected(bool connected);

    /// @brief Set controller state
    MotorFlags setState(ControllerState state);

    /// @brief Get current controller state
    ControllerState getState() const;

    /// @brief Set drive mode
    MotorFlags setMode(DriveMode mode);

    /// @brief Get current drive mode
    DriveMode getMode() const;

    /// @brief Set target motor velocity
    MotorFlags setTargetMotorVelocity(float v);

    /// @brief Set target motor current percentage
    MotorFlags setTargetMotorCurrentPercentage(float c);

    /// @brief Set regenerative braking multiplier
    MotorFlags setMotorRegenMultiplier(float multiplier);

    /// @brief Enable motor output
    MotorFlags turnOn();

    /// @brief Disable motor output
    MotorFlags turnOff();

    /// @brief Toggle cruise control state
    MotorFlags toggleCruise();

    /// @brief Enable or disable cruise control explicitly
    MotorFlags setCruise(bool cruise);

    /// @brief Cycle to the next drive mode
    MotorFlags swapNextDriveMode();

    /// @brief Set an error flag
    MotorFlags setError(MotorFlags error);

    /// @brief Get current error flags
    MotorFlags getError() const;

    /// @brief Clear all error flags
    MotorFlags clearError();

    /// @brief Update internal time references
    void setTime(uint32_t millis, uint32_t micros);

    /// @brief Get current vehicle velocity
    float getVelocity() const;

    /// @brief Get odometer value
    float getOdometer() const;

    /// @brief Get heatsink temperature
    float getHeatSinkTemp() const;

    /// @brief Get motor temperature
    float getMotorTemp() const;

    /// @brief Get DSP board temperature
    float getDspBoardTemp() const;

    /// @brief Set measured bus current
    void setBusCurrent(float value);

    /// @brief Set measured vehicle velocity
    void setVelocity(float value);

    /// @brief Set measured heatsink temperature
    void setHeatSinkTemp(float value);

    /// @brief Set measured motor temperature
    void setMotorTemp(float value);

    /// @brief Set measured DSP board temperature
    void setDspBoardTemp(float value);

    /// @brief Set odometer value
    void setOdometer(float value);

    /// @brief Get target motor velocity
    float getTargetVelocity() const;

    /// @brief Get target motor current
    float getTargetCurrent() const;

    /// @brief Get regenerative braking multiplier
    float getRegenMultiplier() const;

    /// @brief Generate the current motor command
    /// @return SpeedCommand containing desired current and velocity
    SpeedCommand motorCommand();

    /// @brief Set active limit flags
    uint16_t setLimit(uint16_t limit);

    /// @brief Get active limit flags
    uint16_t getLimit() const;

    /// @brief Access button input state (read-only)
    const Buttons &getButtons() const;

    /// @brief Access button input state (mutable)
    Buttons &getButtonsMut();

    /// @brief Access slider input state (read-only)
    const ControllerSliders &getSliders() const;

    /// @brief Access slider input state (mutable)
    ControllerSliders &getSlidersMut();

    /// @brief Set measured motor velocity
    void setMotorVelocity(float velocity);

    /// @brief Get measured motor velocity
    float getMotorVelocity() const;

private:
    /// @brief Startup state handler
    void stateStartup();

    /// @brief Running state handler
    void stateRunning();

    /// @brief Parking state handler
    void stateParking();

    /// @brief Error state handler
    void stateError();

    /// @brief Current control drive mode
    void stateCurrentDrive();

    /// @brief Velocity control drive mode
    void stateVelocityDrive();

    /// @brief Free-wheel (no torque) mode
    void stateFreeWheel();

    /// @brief Cruise control mode
    void stateCruise();

    /// @brief Custom / experimental drive mode
    void stateCustom1();

    ControllerState state;
    DriveMode driveMode;
    DriveMode lastDriveMode;
    MotorDirection direction;

    Buttons buttons;
    ControllerSliders sliders;

    uint32_t lastSentDriveCommand;
    uint32_t currentTime;
    uint32_t lastTime;
    uint32_t deltaTime;
    uint32_t timeSinceMotorDataReceived;

    float regenMultiplier;
    float targetMotorCurrent;
    float targetMotorVelocity;
    float motorVelocity;
    float velocity;
    float current;

    float motorTemp;
    float heatSinkTemp;
    float dspBoardTemp;
    float odometer;

    MotorFlags error;
    uint16_t limit;

    bool motorSettingsChanged;
    bool motorControllerConnected;
};

#endif // CONTROLLER_HPP
