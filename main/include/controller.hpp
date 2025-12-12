//# Copyright (c) 2025 MDU Solar Team
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <stdint.h>
#include <error.hpp>
#include <input.hpp>

#define STARTUP_DRIVE_MODE DriveMode::Current

enum class MotorDirection : int8_t {
  Forward = 1,
  Neutral = 0,
  Backward = -1
};

enum class ControllerState : int8_t {
  Startup,
  Running,
  Parking,
  Error
};

enum class DriveMode: int8_t {
  Current,
  Velocity,
  Cruise,
  Custom1
};

struct SpeedCommand {
  float current;
  float velocity;
};

//TODO make options for slider mode
using ControllerSliders = SmoothSliders;
//using ControllerSliders = Sliders;

class CANSerialState;

class Controller {
public:
  Controller():
    state(ControllerState::Startup),
    driveMode(STARTUP_DRIVE_MODE),
    lastDriveMode(STARTUP_DRIVE_MODE),
    direction(MotorDirection::Forward),
    buttons(),
    sliders(),
    lastSentDriveCommand(0),
    currentTime(0),
    lastTime(0),
    deltaTime(0),
    timeSinceMotorDataReceived(0),
    regenMultiplier(0.0),
    targetMotorCurrent(0.0),
    targetMotorVelocity(0.0),
    motorVelocity(0.0),
    velocity(0.0),
    current(0.0),
    motorTemp(0.0),
    heatSinkTemp(0.0),
    dspBoardTemp(0.0),
    odometer(0.0),
    error(E_SUCCESS),
    limit(0),
    motorSettingsChanged(false),
    motorControllerConnected(false) {}

  void update(unsigned long millis, unsigned long micros);
  bool isStationary(float margin) const;
  bool isInDriveMode() const;
  bool isAccelerometerOff() const;
  
  ControllerResult toggleDirection();
  ControllerResult setDirection(MotorDirection direction);
  MotorDirection getDirection() const;
  ControllerResult setMotorControllerConnected(bool connected);
  ControllerResult setState(ControllerState state);
  ControllerState getState() const;
  ControllerResult setMode(DriveMode mode);
  DriveMode getMode() const;
  ControllerResult setTargetMotorVelocity(float v);
  ControllerResult setTargetMotorCurrentPercentage(float c);
  ControllerResult setMotorRegenMultiplier(float multiplier);
  ControllerResult turnOn();
  ControllerResult turnOff();
  ControllerResult toggleCruise();
  ControllerResult setCruise(bool cruise);
  ControllerResult swapNextDriveMode();
  ControllerResult setError(ControllerResult error);
  ControllerResult getError() const;
  ControllerResult clearError();

  float getVelocity() const;
  float getOdometer() const;
  float getHeatSinkTemp() const;
  float getMotorTemp() const;
  float getDspBoardTemp() const;

  void setBusCurrent(float value);
  void setVelocity(float value);
  void setHeatSinkTemp(float value);
  void setMotorTemp(float value);
  void setDspBoardTemp(float value);
  void setOdometer(float value);

  float getTargetVelocity() const;
  float getTargetCurrent() const;
  float getRegenMultiplier() const;

  SpeedCommand motorCommand();

  uint16_t setLimit(uint16_t limit);
  uint16_t getLimit() const;

  const Buttons& getButtons() const;
  Buttons& getButtonsMut();
  const ControllerSliders& getSliders() const;
  ControllerSliders& getSlidersMut();

  void setMotorVelocity(float velocity);
  float getMotorVelocity() const;

private:
  void stateStartup();
  void stateRunning();
  void stateParking();
  void stateError();
  void stateCurrentDrive();
  void stateVelocityDrive();
  void stateFreeWheel();
  void stateCruise();
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
  ControllerResult error;
  uint16_t limit;
  bool motorSettingsChanged;
  bool motorControllerConnected;
};

#endif //CONTROLLER_HPP