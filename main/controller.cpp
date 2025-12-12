//# Copyright (c) 2025 MDU Solar Team
#include "controller.hpp"

template<typename T>
T max(T a, T b) {
	if (a < b) {
		return b;
	} 
	return a;
}

template<typename T>
T min(T a, T b) {
	if (a < b) {
		return b;
	} 
	return a;
}

template<typename T>
T abs(T a) {
	if (a > 0) {
		return a;
	}
	return -a;
}

const int numDriveModes = 3;
const DriveMode driveModes[numDriveModes] = {DriveMode::Current, DriveMode::Velocity, DriveMode::Custom1};

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
	return {.current = targetMotorCurrent, .velocity = targetMotorVelocity*int(direction)};
}

void Controller::update(unsigned long millis, unsigned long micros) {
	currentTime = millis;
	deltaTime = micros - lastTime;
	lastTime = micros;

	timeSinceMotorDataReceived += deltaTime;
	
	int c = sliders.get(REGENERATION_POTENTIOMETER);
	setMotorRegenMultiplier(float(c) / MAX_POT_VALUE);
	c = sliders.get(ACCELERATION_POTENTIOMETER);
	c = max(c - ACCELERATION_CUTOFF, 0);
	sliders.set(ACCELERATION_POTENTIOMETER, c);

	ControllerResult result = E_SUCCESS;
	if(timeSinceMotorDataReceived > (uint32_t)MOTOR_TIMEOUT*1000) { //timeSinceMotorDataReceived is in micros while MOTOR_TIMEOUT is in millis
		result += setMotorControllerConnected(false);
	} else {
		result += setMotorControllerConnected(true);
	}

	if(buttons.get(NEUTRAL_DIRECTION_SWITCH_BIT)) {
		result += setDirection(MotorDirection::Neutral);
	}
	else if(buttons.get(FORWARD_DIRECTION_SWITCH_BIT)) {
		result += setDirection(MotorDirection::Forward);
	}
	else if(buttons.get(BACKWARD_DIRECTION_SWITCH_BIT)) {
		result += setDirection(MotorDirection::Backward);
	}

	if(result != E_SUCCESS) {
		setState(ControllerState::Error);
		setError(result);
	}

	if(buttons.getJustPressed(CRUISE_SWITCH_BIT)) {
		ControllerResult result = toggleCruise();
		if (result != E_SUCCESS) {
			setState(ControllerState::Error);
		}
	}
	if(buttons.getJustPressed(DRIVE_MODE_SWITCH_BIT)) {
		swapNextDriveMode();
	}
	
	if(heatSinkTemp > MAX_TEMP || dspBoardTemp > MAX_TEMP || motorTemp > MAX_TEMP) {
		setError(E_OVERHEAT);
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
	return sliders.get(ACCELERATION_POTENTIOMETER) <= 0;
}

ControllerResult Controller::toggleDirection() {
	if (direction == MotorDirection::Forward) {
		return setDirection(MotorDirection::Backward);
	} else {
		return setDirection(MotorDirection::Forward);
	}
}

MotorDirection Controller::getDirection() const {
	return direction;
}

ControllerResult Controller::setDirection(MotorDirection direction) {
	if (this->direction == direction) {
		return E_SUCCESS;
	}
	if (isStationary() && isAccelerometerOff()) {
		this->direction = direction;
		return E_SUCCESS;
	}
	return setError(E_NOT_STATIONARY_BIT);
}

ControllerResult Controller::setMotorControllerConnected(bool connected) {
	if (motorControllerConnected == connected) {
		return E_SUCCESS;
	}

	if (isStationary() && isAccelerometerOff()) {
		motorControllerConnected = connected;
		return E_SUCCESS;
	} else {
		return setError(E_CAN);
	}
}

ControllerState Controller::getState() const {
	return state;
}

ControllerResult Controller::setState(ControllerState state) {
	switch (state) {
		case ControllerState::Startup:
			return setError(E_CANNOT_SET_MODE_BIT);
		case ControllerState::Error:
			this->state = ControllerState::Error;
			break;
		case ControllerState::Running:
			if (!isStationary() && isAccelerometerOff()) return setError(E_NOT_STATIONARY_BIT);
			this->state = ControllerState::Running;
			break;
		case ControllerState::Parking:
			if (!isStationary() && isAccelerometerOff()) return setError(E_NOT_STATIONARY_BIT);
			this->state = ControllerState::Parking;
			break;
	}

	return E_SUCCESS;
}

DriveMode Controller::getMode() const {
	return driveMode;
}

//TODO Saftey
ControllerResult Controller::setMode(DriveMode driveMode) {
	this->driveMode = driveMode;
	return E_SUCCESS;
}

ControllerResult Controller::setTargetMotorVelocity(float v) {
	if(v < 0) return setError(E_NEGATIVE_FLOAT_BIT);
	
	if (direction == MotorDirection::Forward)
		targetMotorVelocity = min(v, MAX_FORWARD_VELOCITY);
	else if(direction == MotorDirection::Backward)
		targetMotorVelocity = min(v, MAX_BACKWARD_VELOCITY);
	else
		targetMotorVelocity = 0.0f;

	return E_SUCCESS;
}

ControllerResult Controller::setTargetMotorCurrentPercentage(float c) {
	if (c < 0.0 || c > 1.0) {
		return setError(E_PERCENTAGE_OUT_OF_RANGE_BIT);
	}

	targetMotorCurrent = c;
	return E_SUCCESS;
}

ControllerResult Controller::setMotorRegenMultiplier(float multiplier) {
	if (multiplier < 0.0 || multiplier > 1.0) {
		return setError(E_PERCENTAGE_OUT_OF_RANGE_BIT);
	}
	regenMultiplier = multiplier;
	return E_SUCCESS;
}

ControllerResult Controller::toggleCruise() {
	return setCruise(driveMode != DriveMode::Cruise);
}

ControllerResult Controller::setCruise(bool cruise) {
	DriveMode currentMode = driveMode;
	ControllerResult result = E_SUCCESS;
	if(!cruise && driveMode == DriveMode::Cruise) {
		setMode(lastDriveMode);
	} else if(driveMode != DriveMode::Cruise && (result = setMode(DriveMode::Cruise))) {
		lastDriveMode = currentMode;
		setTargetMotorVelocity(motorVelocity);
	}

	return result;
}

ControllerResult Controller::swapNextDriveMode() {
	int index = -1;
	for (int i = 0; i < numDriveModes; i++) {
		if (driveModes[i] == driveMode) {
			index = i;
			break;
		}
	}
	if (index == -1) {
		return setError(E_WRONG_DRIVE_MODE_BIT);
	}
	return setMode(driveModes[(index+1) %numDriveModes ]);
}

ControllerResult Controller::setError(ControllerResult error) {
	this->error |= error;
	if (this->error != E_SUCCESS)
		setState(ControllerState::Error);
	return error;
}

ControllerResult Controller::getError() const {
	return error;
}

ControllerResult Controller::clearError() {
	//TODO make sure error condition is removed
	error = 0;
	return error;
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
	static unsigned long timeout = currentTime + 2000; //2 seconds
	if (timeout < currentTime) {
		setError(0xffff); //TODO make error 32 bit so that we can encode a error state for startup timeout
		setState(ControllerState::Error);
	}

	if (!motorControllerConnected)
		return;
	if (!isAccelerometerOff())
		return;
	if (!isStationary())
		return;
		//TODO Uncomment
	//if (direction != MotorDirection::Neutral)
		//return;

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
		setTargetMotorCurrentPercentage((float(sliders.get(ACCELERATION_POTENTIOMETER)) / MAX_POT_VALUE) * MAX_FORWARD_CURRENT);
		setTargetMotorVelocity(MAX_FORWARD_VELOCITY);
	}
	else if (direction == MotorDirection::Backward) {
		setTargetMotorCurrentPercentage((float(sliders.get(ACCELERATION_POTENTIOMETER)) / MAX_POT_VALUE) * MAX_BACKWARD_CURRENT);
		setTargetMotorVelocity(MAX_BACKWARD_VELOCITY);
	}  
}

void Controller::stateVelocityDrive() {  
	float maxCurrent;
	if (direction == MotorDirection::Forward) {
		setTargetMotorVelocity((float(sliders.get(ACCELERATION_POTENTIOMETER)) / MAX_POT_VALUE) * MAX_FORWARD_VELOCITY);
		maxCurrent = MAX_FORWARD_CURRENT;
	}
	else if (direction == MotorDirection::Backward) {
		setTargetMotorVelocity((float(sliders.get(ACCELERATION_POTENTIOMETER)) / MAX_POT_VALUE) * MAX_BACKWARD_VELOCITY);
		maxCurrent = MAX_BACKWARD_CURRENT;
	}
	if (abs(motorVelocity) < targetMotorVelocity) {
		setTargetMotorCurrentPercentage(maxCurrent);
	} else {
		setTargetMotorCurrentPercentage(maxCurrent * regenMultiplier);
	}
}

void Controller::stateCruise() {
	static unsigned long timeHeld = 0;
	
	if(buttons.get(CRUISE_ACCELERATE_SWITCH_BIT) && !buttons.get(CRUISE_DEACCELERATE_SWITCH_BIT)) {
		timeHeld += deltaTime;
		setTargetMotorVelocity(targetMotorVelocity + timeHeld / CRUISE_VELOCITY_TIME_TO_CHANGE);
		timeHeld = timeHeld % CRUISE_VELOCITY_TIME_TO_CHANGE;
	} else if(!buttons.get(CRUISE_ACCELERATE_SWITCH_BIT) && buttons.get(CRUISE_DEACCELERATE_SWITCH_BIT)) {
		timeHeld += deltaTime;
		setTargetMotorVelocity(max(targetMotorVelocity - timeHeld / CRUISE_VELOCITY_TIME_TO_CHANGE, 0.0f));
		timeHeld = timeHeld % CRUISE_VELOCITY_TIME_TO_CHANGE;
	} else {
		timeHeld = 0;
	}

	if (abs(motorVelocity) < targetMotorVelocity) {
		setTargetMotorCurrentPercentage(MAX_FORWARD_CURRENT);
	} else {
		setTargetMotorCurrentPercentage(MAX_FORWARD_CURRENT * regenMultiplier);
	}
}

void Controller::stateCustom1() {
	uint16_t pedal = sliders.get(ACCELERATION_POTENTIOMETER);
	uint16_t mid = MAX_POT_VALUE / 2;
	uint16_t dead = POT_DEAD_ZONE_THRESHOLD;

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
