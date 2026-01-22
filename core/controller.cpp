//# Copyright (c) 2025 MDU Solar Team
#include "controller.hpp"
#include <cmath>
#include <iostream>

constexpr float POT_DEAD_ZONE_THRESHOLD = 5.0f;

#define F32_EPSILON 0.0001f //Just has to be a small number in order to be used for comparing f32 values
#define ACCELERATION_POTENTIOMETER 0
#define REGENERATION_POTENTIOMETER 1
#define ACCELERATION_CUTOFF 50
#define WHEEL_RADUIS 0.274f

#define STARTUP_DRIVE_MODE DriveMode::Current
#define MAX_FORWARD_CURRENT 1.0f //Percentage max current on controller
#define MAX_BACKWARD_CURRENT 1.0f //Percentage max current on controller
#define MAX_FORWARD_VELOCITY 400.0f //rpm
#define MAX_BACKWARD_VELOCITY 200.0f //rpm
#define MAX_TEMP 80.0f

#define IS_STATIONARY_MARGIN 0.001f
#define CRUISE_VELOCITY_TIME_TO_CHANGE 100000 //Time in microseconds
#define MOTOR_TIMEOUT 4000

const int numDriveModes = 3;
const DriveMode driveModes[numDriveModes] = {DriveMode::Current, DriveMode::Velocity, DriveMode::Custom1};

Controller::Controller()
	: state(ControllerState::Startup),
		driveMode(STARTUP_DRIVE_MODE),
		lastDriveMode(STARTUP_DRIVE_MODE),
		direction(MotorDirection::Neutral),
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
		error(MotorFlags::Success),
		limit(0),
		motorSettingsChanged(false),
		motorControllerConnected(false) 
	{}

void Controller::setMotorVelocity(float velocity) {
	motorVelocity = velocity;
	timeSinceMotorDataReceived = 0;
}

float Controller::getMotorVelocity() const {
	return motorVelocity;
}

float Controller::getTargetVelocity() const {
	return targetMotorVelocity;
}

float Controller::getTargetCurrent() const {
	return targetMotorCurrent;
}

float Controller::getRegenMultiplier() const {
	return regenMultiplier;
}

SpeedCommand Controller::motorCommand() {
	SpeedCommand command;
	command.current = targetMotorCurrent;
	command.velocity = targetMotorVelocity * int(direction);
	return command;
}

void Controller::setTime(uint32_t millis, uint32_t micros) {
	lastTime = currentTime;
	currentTime = micros;
}

void Controller::update(uint32_t millis, uint32_t micros) {
	//Clear errors that are no longer valid
	clearError();

	currentTime = millis;
	deltaTime = micros - lastTime;
	lastTime = micros;

	timeSinceMotorDataReceived += deltaTime;
	
	float c = sliders.getFloat(REGENERATION_POTENTIOMETER);
	setMotorRegenMultiplier(c);

	MotorFlags result = MotorFlags::Success;
	if(timeSinceMotorDataReceived > (uint32_t)MOTOR_TIMEOUT*10000) { //timeSinceMotorDataReceived is in micros while MOTOR_TIMEOUT is in millis
		result |= setMotorControllerConnected(false);
		setError(MotorFlags::Timeout);
		std::cout << "Delta time: " << deltaTime << std::endl;
		std::cout << "Motor Controller Timeout" << timeSinceMotorDataReceived << ":" << MOTOR_TIMEOUT*10000 << std::endl;
	} else {
		result |= setMotorControllerConnected(true);
	}

	if(buttons.get(DASHBOARD_BUTTON_NEUTRAL)) {
		result |= setDirection(MotorDirection::Neutral);
	}
	else if(buttons.get(DASHBOARD_BUTTON_FORWARD)) {
		result |= setDirection(MotorDirection::Forward);
	}
	else if(buttons.get(DASHBOARD_BUTTON_REVERSE)) {
		result |= setDirection(MotorDirection::Backward);
	}

	if(result != MotorFlags::Success) {
		setError(result);
		std::cout << "Error setting direction: " << static_cast<int>(result) << std::endl;
	}

	if(buttons.getJustPressed(DASHBOARD_BUTTON_CRUISE_CONTROL)) {
		MotorFlags result = toggleCruise();
		if (result != MotorFlags::Success) {
			std::cout << "Error toggling cruise: " << static_cast<int>(result) << std::endl;
		}
	}
	if(buttons.getJustPressed(DASHBOARD_BUTTON_CRUISE_CONTROL)) {
		swapNextDriveMode();
	}
	
	if(heatSinkTemp > MAX_TEMP || dspBoardTemp > MAX_TEMP || motorTemp > MAX_TEMP) {
		setError(MotorFlags::Overheat);
	}

	switch (state) {
		case ControllerState::Startup:
			stateStartup();
			break; 
		case ControllerState::Running:
			stateRunning();
			break;
		case ControllerState::Parking:
			stateParking();
			break;
		case ControllerState::Error:
			stateError();
			break;
		default:
			stateError();
			break;
	}
}

bool Controller::isStationary(float margin = IS_STATIONARY_MARGIN) const {
	return abs(motorVelocity) <= margin;
}

bool Controller::isInDriveMode() const {
	for (int i = 0; i < numDriveModes; i++) {
		if (driveMode == driveModes[i]) {
			return true;
		}
	}
	return false;
}

bool Controller::isAccelerometerOff() const {
	return sliders.getFloat(ACCELERATION_POTENTIOMETER) <= 0.0f + F32_EPSILON;
}

MotorFlags Controller::toggleDirection() {
	if (direction == MotorDirection::Forward) {
		return setDirection(MotorDirection::Backward);
	} else {
		return setDirection(MotorDirection::Forward);
	}
}

MotorDirection Controller::getDirection() const {
	return direction;
}

MotorFlags Controller::setDirection(MotorDirection direction) {
	if (this->direction == direction) {
		return MotorFlags::Success;
	}
	if (isStationary() && isAccelerometerOff()) {
		this->direction = direction;
		return MotorFlags::Success;
	}
	return setError(MotorFlags::NotStationary);
}

MotorFlags Controller::setMotorControllerConnected(bool connected) {
	if (motorControllerConnected == connected) {
		return MotorFlags::Success;
	}

	if (!connected) {
		motorControllerConnected = false;
		return MotorFlags::Success;
	}

	if (!isStationary()) {
		return setError(
			MotorFlags::NotStationary
		);
	};

	if (!isAccelerometerOff()) {
		return setError(
			MotorFlags::NotStationary
		);
	};
	
	motorControllerConnected = connected;
	return MotorFlags::Success;
}

ControllerState Controller::getState() const {
	return state;
}

MotorFlags Controller::setState(ControllerState state) {
	switch (state) {
		case ControllerState::Startup:
			return setError(MotorFlags::CannotSetMode);
		case ControllerState::Error:
			this->state = ControllerState::Error;
			std::cout << "Controller entered Error state" << std::endl;
			break;
		case ControllerState::Running:
			if (!isStationary() && isAccelerometerOff()) return setError(MotorFlags::NotStationary);
			this->state = ControllerState::Running;
			break;
		case ControllerState::Parking:
			if (!isStationary() && isAccelerometerOff()) return setError(MotorFlags::NotStationary);
			this->state = ControllerState::Parking;
			break;
	}

	return MotorFlags::Success;
}

DriveMode Controller::getMode() const {
	return driveMode;
}

//TODO Saftey
MotorFlags Controller::setMode(DriveMode driveMode) {
	this->driveMode = driveMode;
	return MotorFlags::Success;
}

MotorFlags Controller::setTargetMotorVelocity(float v) {
	if(v < 0) return setError(MotorFlags::NegativeFloat);
	
	if (direction == MotorDirection::Forward)
		targetMotorVelocity = std::min(v, MAX_FORWARD_VELOCITY);
	else if(direction == MotorDirection::Backward)
		targetMotorVelocity = std::min(v, MAX_BACKWARD_VELOCITY);
	else
		targetMotorVelocity = 0.0f;

	return MotorFlags::Success;
}

MotorFlags Controller::setTargetMotorCurrentPercentage(float c) {
	if (c < 0.0 || c > 1.0) {
		return setError(MotorFlags::PercentageOutOfRange);
	}

	targetMotorCurrent = c;
	return MotorFlags::Success;
}

MotorFlags Controller::setMotorRegenMultiplier(float multiplier) {
	if (multiplier < 0.0 || multiplier > 1.0) {
		return setError(MotorFlags::PercentageOutOfRange);
	}
	regenMultiplier = multiplier;
	return MotorFlags::Success;
}

MotorFlags Controller::toggleCruise() {
	return setCruise(driveMode != DriveMode::Cruise);
}

MotorFlags Controller::setCruise(bool cruise) {
	DriveMode currentMode = driveMode;
	MotorFlags result = MotorFlags::Success;
	if(!cruise && driveMode == DriveMode::Cruise) {
		setMode(lastDriveMode);
	} else if(driveMode != DriveMode::Cruise && (result = setMode(DriveMode::Cruise)) == MotorFlags::Success) {
		lastDriveMode = currentMode;
		setTargetMotorVelocity(motorVelocity);
	}

	return result;
}

MotorFlags Controller::swapNextDriveMode() {
	int index = -1;
	for (int i = 0; i < numDriveModes; i++) {
		if (driveModes[i] == driveMode) {
			index = i;
			break;
		}
	}
	if (index == -1) {
		return setError(MotorFlags::WrongDriveMode);
	}
	return setMode(driveModes[(index+1) %numDriveModes ]);
}

MotorFlags Controller::setError(MotorFlags error) {
	this->error |= error;
	if (this->error != MotorFlags::Success)
		setState(ControllerState::Error);
	return error;
}

MotorFlags Controller::getError() const {
	return error;
}

MotorFlags Controller::clearError() {
	if (error == MotorFlags::Success)
		return MotorFlags::Success;
	std::cout << "Clearing error state from: " << static_cast<int>(error) << std::endl;
	state = ControllerState::Startup;
	error = MotorFlags::Success;
	return MotorFlags::Success;
}

float Controller::getVelocity() const {
	return velocity;
}

float Controller::getOdometer() const {
	return odometer;
}

float Controller::getHeatSinkTemp() const {
	return heatSinkTemp;
}

float Controller::getMotorTemp() const {
	return motorTemp;
}

float Controller::getDspBoardTemp() const {
	return dspBoardTemp;
}

void Controller::setBusCurrent(float value) {
	current = value;
}

void Controller::setVelocity(float value) {
	velocity = value;
}

void Controller::setHeatSinkTemp(float value) {
	heatSinkTemp = value;
}

void Controller::setMotorTemp(float value) {
	motorTemp = value;
}

void Controller::setDspBoardTemp(float value) {
	dspBoardTemp = value;
}

void Controller::setOdometer(float value) {
	odometer = value;
}

uint16_t Controller::setLimit(uint16_t limit) {
	this->limit = limit;
	return limit;
}

uint16_t Controller::getLimit() const {
	return limit;
}

const Buttons& Controller::getButtons() const {
	return buttons;
}

Buttons& Controller::getButtonsMut() {
	return buttons;
}

const ControllerSliders& Controller::getSliders() const {
	return sliders;
}

ControllerSliders& Controller::getSlidersMut() {
	return sliders;
}

void Controller::stateStartup() {
	if (!motorControllerConnected)
		return;
	if (!isAccelerometerOff())
		return;
	if (!isStationary())
		return;
	if (direction != MotorDirection::Neutral)
		return;

	setState(ControllerState::Running);
}


void Controller::stateParking() {
	setTargetMotorCurrentPercentage(MAX_FORWARD_CURRENT);
	setTargetMotorVelocity(motorVelocity/2.0f);
}

void Controller::stateRunning() {
	if (direction == MotorDirection::Neutral) {
		setTargetMotorCurrentPercentage(0.0f);
		setTargetMotorVelocity(0.0f);
		return;
	}
	switch (driveMode) {
		case DriveMode::Current:
			stateCurrentDrive();
			break;
		case DriveMode::Velocity:
			stateVelocityDrive();
			break;
		case DriveMode::Cruise:
			stateCruise();
			break;
		case DriveMode::Custom1:
			stateCustom1();
			break;
	}
}

void Controller::stateError() {
	//TODO give better error messages
	//TODO write error to display
	setTargetMotorCurrentPercentage(0.0f);
	setTargetMotorVelocity(0.0f);
}

void Controller::stateCurrentDrive() {
	if (direction == MotorDirection::Forward) {
		std::cout << "Forward Current Drive, slider value: " << sliders.getFloat(ACCELERATION_POTENTIOMETER) << std::endl; 
		setTargetMotorCurrentPercentage(sliders.getFloat(ACCELERATION_POTENTIOMETER) * MAX_FORWARD_CURRENT);
		setTargetMotorVelocity(MAX_FORWARD_VELOCITY);
	}
	else if (direction == MotorDirection::Backward) {
		setTargetMotorCurrentPercentage(sliders.getFloat(ACCELERATION_POTENTIOMETER) * MAX_BACKWARD_CURRENT);
		setTargetMotorVelocity(MAX_BACKWARD_VELOCITY);
	}  
}

void Controller::stateVelocityDrive() {  
	float maxCurrent;
	std::cout << "Velocity Drive, slider value: " << sliders.getFloat(ACCELERATION_POTENTIOMETER) << std::endl;
	std::cout << sliders.getFloat(ACCELERATION_POTENTIOMETER) * MAX_FORWARD_VELOCITY << std::endl;
	if (direction == MotorDirection::Forward) {
		setTargetMotorVelocity(sliders.getFloat(ACCELERATION_POTENTIOMETER) * MAX_FORWARD_VELOCITY);
		maxCurrent = MAX_FORWARD_CURRENT;
		std::cout << "Target Velocity: " << targetMotorVelocity << std::endl;
	}
	else if (direction == MotorDirection::Backward) {
		setTargetMotorVelocity(sliders.getFloat(ACCELERATION_POTENTIOMETER) * MAX_BACKWARD_VELOCITY);
		maxCurrent = MAX_BACKWARD_CURRENT;
		std::cout << "Target Velocity: " << targetMotorVelocity << std::endl;
	}
	if (abs(motorVelocity) < targetMotorVelocity) {
		setTargetMotorCurrentPercentage(maxCurrent);
		std::cout << "Setting max current for acceleration: " << maxCurrent << std::endl;
	} else {
		setTargetMotorCurrentPercentage(maxCurrent * regenMultiplier);
		std::cout << "Setting regen current: " << maxCurrent * regenMultiplier << std::endl;
	}
}

void Controller::stateCruise() {
	if (abs(motorVelocity) < targetMotorVelocity) {
		setTargetMotorCurrentPercentage(MAX_FORWARD_CURRENT);
	} else {
		setTargetMotorCurrentPercentage(MAX_FORWARD_CURRENT * regenMultiplier);
	}
}

void Controller::stateCustom1() {
	float pedal = sliders.getFloat(ACCELERATION_POTENTIOMETER);
	float mid = 0.5f;
	float dead = 0.05f;

	if (pedal > mid + dead) {      // accelerate
		float frac = (pedal - mid) / mid;
		setTargetMotorVelocity(MAX_FORWARD_VELOCITY);
		setTargetMotorCurrentPercentage(frac * MAX_FORWARD_CURRENT);
	}
	else if (pedal < mid - dead) { // regen
		float frac = (mid - pedal) / mid;
		setTargetMotorVelocity(0);
		setTargetMotorCurrentPercentage(frac * regenMultiplier);
	}
}

void Controller::processIncomingCommand(const CanFrame &frame)
{
    switch (frame.id)
    {
    case ID_STATUS_INFORMATION:
    {
        StatusInformation status(frame);
        setError(static_cast<MotorFlags>(status.error_flags & 0x1ff)); // mask to remove unwanted reserved bits
        setLimit(status.limit_flags & 0x7f);                           // mask to remove unwanted reserved bits
        break;
    }
    case ID_BUS_MEASUREMENT:
    {
        BusMeasurement bm(frame);
        setBusCurrent(bm.bus_current);
        break;
    }
    case ID_VELOCITY_MEASUREMENT:
    {
        VelocityMeasurement vm(frame);
        setVelocity(vm.vehicle_velocity);
        setMotorVelocity(vm.motor_velocity_rpm);
        break;
    }
    case ID_PHASE_CURRENT_MEASUREMENT:
    {
        PhaseCurrentMeasurement pcm(frame);
        break;
    }
    case ID_MOTOR_VOLTAGE_VECTOR_MEASUREMENT:
    {
        MotorVoltageVectorMeasurement vvm(frame);
        break;
    }
    case ID_MOTOR_CURRENT_VECTOR_MEASUREMENT:
    {
        MotorCurrentVectorMeasurement cvm(frame);

        break;
    }
    case ID_HEATSINK_AND_MOTOR_TEMPERATURE_MEASUREMENT:
    {
        HeatsinkAndMotorTemperatureMeasurement temp(frame);
        setHeatSinkTemp(temp.heat_sink_temp);
        setMotorTemp(temp.motor_temp);
        break;
    }
    case ID_DSP_BOARD_TEMPERATURE_MEASUREMENT:
    {
        DSPBoardTemperatureMeasurement temp(frame);
        setDspBoardTemp(temp.DSP_board_temp);
        break;
    }
    case ID_ODOMETER_AND_BUS_AMP_HOURS_MEASUREMENT:
    {
        OdometerAndBusAmpHoursMeasurement om(frame);
        setOdometer(om.odometer);
        break;
    }
    case ID_SLIP_SPEED_MEASUREMENT:
    {
        SlipSpeedMeasurement ssm(frame);
        break;
    }
	case ID_ACCELEROMETER_PERCENTAGE:
	{
		DriverAccelerometer da(frame);
		sliders.set(ACCELERATION_POTENTIOMETER, da.acceleration);
	}
    default:
        // TODO FIX
        break;
    }
}

void Controller::sendPeriodicMessages(uint32_t current_time_ms, void* t_state, void(*transmit_func)(const CanFrame&, void*)) {
	if (current_time_ms - lastSentDriveCommand < 50) {
		return;
	}

	lastSentDriveCommand = current_time_ms;
	SpeedCommand command = motorCommand();
    MotorDriveCommand cmd(command.current, command.velocity);

	std::cout << "Motor Driver Running..." << std::endl;
	std::cout << "Target Current: " << command.current << " Target Velocity: " << command.velocity << std::endl;
	std::cout << "Actual Velocity: " << getMotorVelocity() << " Odometer: " << getOdometer() << std::endl;
	std::cout << "Heat Sink Temp: " << getHeatSinkTemp() << " Motor Temp: " << getMotorTemp() << " DSP Temp: " << getDspBoardTemp() << std::endl;
	std::cout << "Error Code: " << static_cast<int>(getError()) << " Limit Code: " << getLimit() << std::endl;
	std::cout << "Drive Mode: " << int(getMode()) << " Direction: " << int(getDirection()) << std::endl;
	std::cout << "Controller State: " << int(getState()) << std::endl;
	std::cout << "-----------------------------" << std::endl;

	std::cout << "Sending Command - Current: " << command.current << ", Velocity: " << command.velocity << std::endl;

    (transmit_func)(pack(cmd), t_state);	
}