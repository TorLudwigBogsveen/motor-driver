#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include "can_frame.hpp"
#include <cstdint> // For uint32_t
#include <cstring>  // For std::memcpy

/// @file protocol.hpp
/// @brief CAN protocol definitions for motor controller communication
/// @note Low float first then high float and then MSB first within each float

// ===========================================================
// STATIC ASSERTS TO ENSURE PLATFORM COMPATIBILITY
// ===========================================================

#include <limits>
static_assert(sizeof(float) == 4, "Platform must use 32-bit float for CAN protocol compatibility");
static_assert(std::numeric_limits<float>::is_iec559, "Platform must use IEEE 754 float representation");

// ===========================================================

// BASE ADDRESSES (Configured in the software for Wave Sculptor motor controller)
constexpr uint32_t DRIVER_CONTROLS_BASE_ADDRESS  = 0x500; ///< Base address for driver control commands
constexpr uint32_t MOTOR_CONTROLLER_BASE_ADDRESS = 0x400; ///< Base address for motor controller messages

// CAN Message Identifiers for Drive Commands
constexpr uint32_t ID_MOTOR_DRIVE_COMMAND = DRIVER_CONTROLS_BASE_ADDRESS + 0x01;   ///< Motor Controller Command, WaveSculptor22 motor controller must receive a Motor Drive Command frame at least once every 250ms!
constexpr uint32_t ID_MOTOR_POWER_COMMAND = DRIVER_CONTROLS_BASE_ADDRESS + 0x02;   ///< Motor Power Command
constexpr uint32_t ID_RESET_COMMAND 	  = DRIVER_CONTROLS_BASE_ADDRESS + 0x03;         ///< System Reset Command

// CAN Message Identifiers for Motor Controll Broadcast Messages
constexpr uint32_t ID_IDENTIFICATION_INFORMATION 	= MOTOR_CONTROLLER_BASE_ADDRESS + 0x00; ///< Identification Information (Interval 1000ms)
constexpr uint32_t ID_STATUS_INFORMATION 			= MOTOR_CONTROLLER_BASE_ADDRESS + 0x01;         ///< Status Information (Interval 200ms)
constexpr uint32_t ID_BUS_MEASUREMENT 				= MOTOR_CONTROLLER_BASE_ADDRESS + 0x02; ///< Bus Measurement (Interval 200ms)
constexpr uint32_t ID_VELOCITY_MEASUREMENT 			= MOTOR_CONTROLLER_BASE_ADDRESS + 0x03; ///< Velocity Measurement (Interval 200ms)
constexpr uint32_t ID_PHASE_CURRENT_MEASUREMENT 	= MOTOR_CONTROLLER_BASE_ADDRESS + 0x04; ///< Phase Current Measurement (Interval 200ms)
constexpr uint32_t ID_MOTOR_VOLTAGE_VECTOR_MEASUREMENT 	   = MOTOR_CONTROLLER_BASE_ADDRESS + 0x05; ///< Motor Voltage Vector Measurement (Interval 200ms)
constexpr uint32_t ID_MOTOR_CURRENT_VECTOR_MEASUREMENT	   = MOTOR_CONTROLLER_BASE_ADDRESS + 0x06; ///< Motor Current Vector Measurement (Interval 200ms)
constexpr uint32_t ID_MOTOR_BACK_EMF_MEASUREMENT 		   = MOTOR_CONTROLLER_BASE_ADDRESS + 0x07; ///< Motor Back-EMF Measurement/Prediction (Interval 200ms)
constexpr uint32_t ID_15V_VOLTAGE_RAIL_MEASUREMENT 		   = MOTOR_CONTROLLER_BASE_ADDRESS + 0x08; ///< 15V Voltage Rail Measurement (Interval 1000ms)
constexpr uint32_t ID_3V3_AND_1V9_VOLTAGE_RAIL_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x09; ///< 3.3V and 1.9V Voltage Rail Measurement (Interval 1000ms)
// ID: MOTOR_CONOTROLLER_BASE_ADDRESS + 0x0A is reserved
constexpr uint32_t ID_HEATSINK_AND_MOTOR_TEMPERATURE_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x0B; ///< Heatsink and Motor Temperature Measurement (Interval 1000ms)
constexpr uint32_t ID_DSP_BOARD_TEMPERATURE_MEASUREMENT 		 = MOTOR_CONTROLLER_BASE_ADDRESS + 0x0C; ///< DSP Board Temperature Measurement (Interval 1000ms)
// ID: MOTOR_CONTROLLER_BASE_ADDRESS + 0x0D is reserved
constexpr uint32_t ID_ODOMETER_AND_BUS_AMP_HOURS_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x0E; ///< Odometer and Bus Amp-Hours Measurement (Interval 1000ms)
constexpr uint32_t ID_SLIP_SPEED_MEASUREMENT 				 = MOTOR_CONTROLLER_BASE_ADDRESS + 0x17; ///< Slip Speed Measurement (Interval 200ms)

// CAN Message Identifiers for Configuration Commands
constexpr uint32_t ID_ACTIVE_MOTOR_CHANGE = MOTOR_CONTROLLER_BASE_ADDRESS + 0x12; ///< Active Motor Change, send this command to change the active motor (if multiple motors are configured)

// Forward Declarations of domain structs for CAN messages
struct MotorDriveCommand;
struct MotorPowerCommand;
struct ResetCommand;

struct IdentificationInformation;
struct StatusInformation;
struct BusMeasurement;
struct VelocityMeasurement;
struct PhaseCurrentMeasurement;
struct MotorVoltageVectorMeasurement;
struct MotorCurrentVectorMeasurement;
struct MotorBackEMFMeasurement;
struct VoltageRailMeasurement15V;
struct VoltageRailMeasurement3V3And1V9;
struct HeatsinkAndMotorTemperatureMeasurement;
struct DSPBoardTemperatureMeasurement;
struct OdometerAndBusAmpHoursMeasurement;
struct SlipSpeedMeasurement;

struct ActiveMotorChangeCommand;

// Forward Declarations of serialization functions
CanFrame pack(const MotorDriveCommand& cmd);
CanFrame pack(const MotorPowerCommand& cmd);
CanFrame pack(const ResetCommand& cmd);

CanFrame pack(const IdentificationInformation& info);
CanFrame pack(const StatusInformation& info);
CanFrame pack(const BusMeasurement& measurement);
CanFrame pack(const VelocityMeasurement& measurement);
CanFrame pack(const PhaseCurrentMeasurement& measurement);
CanFrame pack(const MotorVoltageVectorMeasurement& measurement);
CanFrame pack(const MotorCurrentVectorMeasurement& measurement);
CanFrame pack(const MotorBackEMFMeasurement& measurement);
CanFrame pack(const VoltageRailMeasurement15V& measurement);
CanFrame pack(const VoltageRailMeasurement3V3And1V9& measurement);
CanFrame pack(const HeatsinkAndMotorTemperatureMeasurement& measurement);
CanFrame pack(const DSPBoardTemperatureMeasurement& measurement);
CanFrame pack(const OdometerAndBusAmpHoursMeasurement& measurement);
CanFrame pack(const SlipSpeedMeasurement& measurement);

CanFrame pack(const ActiveMotorChangeCommand& cmd);

// ============================================================================
// STRUCT DEFINITIONS
// ============================================================================

/// @brief Drive command message for motor controller
struct MotorDriveCommand {
    float motor_current_percent;  ///< Motor current percentage (0-100%, always positive)
    float motor_velocity_rpm;   ///< Target velocity in RPM (positive for forward, negative for reverse)
    
    /// @brief Construct from CAN frame (deserialize)
    explicit MotorDriveCommand(const CanFrame& frame);
    
    /// @brief Default constructor  
    MotorDriveCommand() : motor_current_percent{0}, motor_velocity_rpm{0} {}
};

/// @brief Power command message for motor controller
/// @note Low float is reserved in this case, so only high float is used
struct MotorPowerCommand {
	float bus_current;  ///< Bus current in percents (0-100%)
	// The low float is reserved

	/// @brief Construct from CAN frame (deserialize)
	explicit MotorPowerCommand(const CanFrame& frame);
	
	/// @brief Default constructor  
	MotorPowerCommand() : bus_current{0} {}
};

/// @brief Reset command message for motor controller
/// @note Send a command from this address to reset the software in the WaveSculptor, not used during normal operation but can be used to reset the device if necessary
struct ResetCommand {
	// No used variables

	/// @brief Construct from CAN frame (deserialize)
	explicit ResetCommand(const CanFrame& frame);

	/// @brief Default constructor by compiler
	ResetCommand() = default;
};

/// @brief Identification information message from motor controller
/// @note Contains information that should be constant (Device serial e.g), I'll fix later
struct IdentificationInformation {
	uint32_t serial_number;	///< Device serial number of the motor controller (allocated at manufacture)
	uint32_t prohelion_ID;	///< Device identifier

	/// @brief Construct from CAN frame (deserialize)
	explicit IdentificationInformation(const CanFrame& frame);

	/// @brief Default constructor
	IdentificationInformation() : serial_number{0}, prohelion_ID{0} {}
};

/// @brief Status information message from motor controller
struct StatusInformation {
	uint8_t receive_error_count;  ///< The DSP CAN receive error counter (CAN 2.0)
	uint8_t transmit_error_count; ///< The DSP CAN transmission error counter (CAN 2.0)
	uint16_t active_motor; ///< The index of the active motor currently being used
	uint16_t error_flags; ///< Error status flags (16-bit field):
	                      ///< Bits 15-9: Reserved
	                      ///< Bit 8: Motor Over Speed (15% overshoot above max RPM)
	                      ///< Bit 7: Desaturation Fault (IGBT desaturation, IGBT driver OVLO)
	                      ///< Bit 6: 15V Rail under voltage lock out (UVLO)
	                      ///< Bit 5: Config read error (some values may be reset to defaults)
	                      ///< Bit 4: Watchdog caused last reset
	                      ///< Bit 3: Bad motor position hall sequence
	                      ///< Bit 2: DC Bus over voltage
	                      ///< Bit 1: Software over current
	                      ///< Bit 0: Hardware over current

	uint16_t limit_flags; ///< Limit status flags (16-bit field) - indicate which control loop is limiting the output current:
	                      ///< Bits 15-7: Reserved
	                      ///< Bit 6: IPM Temperature or Motor Temperature
	                      ///< Bit 5: Bus Voltage Lower Limit
	                      ///< Bit 4: Bus Voltage Upper Limit
	                      ///< Bit 3: Bus Current
	                      ///< Bit 2: Velocity
	                      ///< Bit 1: Motor Current
	                      ///< Bit 0: Output Voltage PWM

	/// @brief Construct from CAN frame (deserialize)
	explicit StatusInformation(const CanFrame& frame);

	/// @brief Default constructor
	StatusInformation() : receive_error_count{0}, transmit_error_count{0}, active_motor{0}, error_flags{0}, limit_flags{0} {}
};

/// @brief Bus measurement message from motor controller
struct BusMeasurement {
	float bus_current;  ///< (Units A) Current drawn from the DC bus by the controller.
	float bus_voltage;   ///< (Units V) DC bus voltage at the controller.

	/// @brief Construct from CAN frame (deserialize)
	explicit BusMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	BusMeasurement() : bus_current{0}, bus_voltage{0} {}
};

/// @brief Velocity measurement message from motor controller
struct VelocityMeasurement {
	float vehicle_velocity; ///< (Units m/s) Vehicle velocity
	float motor_velocity_rpm;  ///< (Units RPM) Motor angular frequency in revolutions per minute

	/// @brief Construct from CAN frame (deserialize)
	explicit VelocityMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	VelocityMeasurement() : vehicle_velocity{0}, motor_velocity_rpm{0} {}
};

/// @brief Phase current measurement message from motor controller
/// @note While the motor is rotating at speed these two currents should be equal. At extremely low commutation speeds these two currents will only match in one third of the motor position, the other two thirds will involve current also flowing in Phase A.
struct PhaseCurrentMeasurement {
	float phase_c_current; ///< (Units A_rms) RMS current in motor Phase C
	float phase_b_current; ///< (Units A_rms) RMS current in motor Phase B

	/// @brief Construct from CAN frame (deserialize)
	explicit PhaseCurrentMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	PhaseCurrentMeasurement() : phase_c_current{0}, phase_b_current{0} {}
};

/// @brief Motor voltage vector measurement message from motor controller
struct MotorVoltageVectorMeasurement {
	float v_d; ///< (Units V) Real component of the applied non-rotating voltage vector to the motor.
	float v_q; ///< (Units V) Imaginary component of the applied non-rotating voltage vector to the motor.

	/// @brief Construct from CAN frame (deserialize)
	explicit MotorVoltageVectorMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	MotorVoltageVectorMeasurement() : v_d{0}, v_q{0} {}
};

/// @brief Motor current vector measurement message from motor controller
struct MotorCurrentVectorMeasurement {
	float i_d; ///< (Units A) Real component of the applied non-rotating current vector to the motor. This vector represents the field current of the motor.
	float i_q; ///< (Units A) Imaginary component of the applied non-rotating current vector to the motor. This current produces torque in the motor and should be in phase with the back-EMF of the motor.

	/// @brief Construct from CAN frame (deserialize)
	explicit MotorCurrentVectorMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	MotorCurrentVectorMeasurement() : i_d{0}, i_q{0} {}
};

/// @brief Motor back-EMF measurement message from motor controller
struct MotorBackEMFMeasurement {
	float BEMf_d; ///< (Units V) By definition this value is always 0V.
	float BEMf_q; ///< (Units V) The peak of the to neutral motor voltage.

	/// @brief Construct from CAN frame (deserialize)
	explicit MotorBackEMFMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	MotorBackEMFMeasurement() : BEMf_d{0}, BEMf_q{0} {}
};

/// @brief 15V voltage rail measurement message from motor controller
struct VoltageRailMeasurement15V {
	float supply_of_15V; ///< (Units V) Actual voltage level of the 15V power rail.	
	// Low float is reserved

	/// @brief Construct from CAN frame (deserialize)
	explicit VoltageRailMeasurement15V(const CanFrame& frame);

	/// @brief Default constructor
	VoltageRailMeasurement15V() : supply_of_15V{0} {}
};

/// @brief 3.3V and 1.9V voltage rail measurement message from motor controller
struct VoltageRailMeasurement3V3And1V9 {
	float supply_of_3V3;  ///< (Units V) Actual voltage level of the 3.3V power rail.
	float supply_of_1V9;  ///< (Units V) Actual voltage level of the 1.9V DSP power rail.

	/// @brief Construct from CAN frame (deserialize)
	explicit VoltageRailMeasurement3V3And1V9(const CanFrame& frame);

	/// @brief Default constructor
	VoltageRailMeasurement3V3And1V9() : supply_of_3V3{0}, supply_of_1V9{0} {}
};

/// @brief Heatsink and motor temperature measurement message from motor controller
struct HeatsinkAndMotorTemperatureMeasurement {
	float heat_sink_temp; ///< (Units °C) Internal temperature of Heat-sink (case).
	float motor_temp; ///< (Units °C) Internal temperature of the motor.

	/// @brief Construct from CAN frame (deserialize)
	explicit HeatsinkAndMotorTemperatureMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	HeatsinkAndMotorTemperatureMeasurement() : heat_sink_temp{0}, motor_temp{0} {}
};

/// @brief DSP board temperature measurement message from motor controller
struct DSPBoardTemperatureMeasurement {
	// High float reserved
	float DSP_board_temp; ///< (Units °C) Temperature of DSP board.

	/// @brief Construct from CAN frame (deserialize)
	explicit DSPBoardTemperatureMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	DSPBoardTemperatureMeasurement() : DSP_board_temp{0} {}
};

/// @brief Odometer and bus amp-hours measurement message from motor controller
struct OdometerAndBusAmpHoursMeasurement {
	float DC_bus_amp_hours; ///< (Units Ah) Charge flow into the controller DC bus from the time of reset.
	float odometer; ///< (Units m) Distance the vehicle has travelled since reset.

	/// @brief Construct from CAN frame (deserialize)
	explicit OdometerAndBusAmpHoursMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	OdometerAndBusAmpHoursMeasurement() : DC_bus_amp_hours{0}, odometer{0} {}
};

/// @brief Slip speed measurement message from motor controller
struct SlipSpeedMeasurement {
	float slip_speed; ///< (Units Hz) Slip speed when driving an induction motor.
	// Low float reserved

	/// @brief Construct from CAN frame (deserialize)
	explicit SlipSpeedMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	SlipSpeedMeasurement() : slip_speed{0} {}
};

/// @brief Active motor change command message for motor controller
/// @note Send this command to change the active motor (if multiple motors are configured)
/// @warning Controller saves active motor to EEPROM - don't send constantly to avoid wearing out EEPROM
struct ActiveMotorChangeCommand {
	uint16_t active_motor; ///< Desired active motor index (0 to 9)
	uint8_t access_key[6]; ///< Configuration access key - must be ASCII "ACTMOT" (0x41 0x43 0x54 0x4D 0x4F 0x54)

	/// @brief Construct from CAN frame (deserialize)
	explicit ActiveMotorChangeCommand(const CanFrame& frame);

	/// @brief Default constructor - sets access key to "ACTMOT"
	ActiveMotorChangeCommand() : active_motor{0}, access_key{'A', 'C', 'T', 'M', 'O', 'T'} {}

	/// @brief Convenience constructor with motor index
	/// @param motor_index Desired active motor (0 to 9)
	explicit ActiveMotorChangeCommand(uint16_t motor_index) : active_motor{motor_index}, access_key{'A', 'C', 'T', 'M', 'O', 'T'} {}
};

// ============================================================================
// DESERIALIZATION IMPLEMENTATIONS (CAN frames -> Domain Structs)
// ============================================================================

inline MotorDriveCommand::MotorDriveCommand(const CanFrame& frame) {
	// Unpack motor_velocity_rpm (IEEE 754 32-bit float) from first 4 bytes - MSB first
	uint32_t velocity_bits { 
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&motor_velocity_rpm, &velocity_bits, sizeof(float));

	// Unpack motor_current_percent from remaining bytes (bytes 4-7 as another IEEE 754 float)
	uint32_t current_bits { 
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&motor_current_percent, &current_bits, sizeof(float));
}

inline MotorPowerCommand::MotorPowerCommand(const CanFrame& frame) {
	// Unpack bus_current from bytes 4-7 (IEEE 754 float) - MSB first
	uint32_t current_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&bus_current, &current_bits, sizeof(float));
}

inline ResetCommand::ResetCommand(const CanFrame& /*frame*/) {
	// No data to unpack
}

inline IdentificationInformation::IdentificationInformation(const CanFrame& frame) {
	// Unpack prohelion_ID from first 4 bytes - MSB first
	prohelion_ID = 
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]));

	// Unpack serial_number from next 4 bytes - MSB first
	serial_number = 
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]));
}

inline StatusInformation::StatusInformation(const CanFrame& frame) {
	limit_flags = 
		(static_cast<uint16_t>(frame.data[0]) << 8) |
		(static_cast<uint16_t>(frame.data[1]));
	
	error_flags = 
		(static_cast<uint16_t>(frame.data[2]) << 8) |
		(static_cast<uint16_t>(frame.data[3]));
	
	active_motor = 
		(static_cast<uint16_t>(frame.data[4]) << 8) |
		(static_cast<uint16_t>(frame.data[5]));
	
	transmit_error_count = frame.data[6]; // Already uint_8 no need for cast

	receive_error_count = frame.data[7]; // Already uint_8 no need for cast
}

inline BusMeasurement::BusMeasurement(const CanFrame& frame) {
	// Unpack bus_voltage from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t voltage_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};	

	std::memcpy(&bus_voltage, &voltage_bits, sizeof(float));

	// Unpack bus_current from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t current_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&bus_current, &current_bits, sizeof(float));
}

inline VelocityMeasurement::VelocityMeasurement(const CanFrame& frame) {
	// Unpack motor_velocity_rpm from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t rpm_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&motor_velocity_rpm, &rpm_bits, sizeof(float));

	// Unpack vehicle_velocity from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t velocity_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&vehicle_velocity, &velocity_bits, sizeof(float));
}

inline PhaseCurrentMeasurement::PhaseCurrentMeasurement(const CanFrame& frame) {
	// Unpack phase_b_current from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t phase_b_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&phase_b_current, &phase_b_bits, sizeof(float));

	// Unpack phase_c_current from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t phase_c_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&phase_c_current, &phase_c_bits, sizeof(float));
}

inline MotorVoltageVectorMeasurement::MotorVoltageVectorMeasurement(const CanFrame& frame) {
	// Unpack v_q from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t v_q_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&v_q, &v_q_bits, sizeof(float));

	// Unpack v_d from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t v_d_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&v_d, &v_d_bits, sizeof(float));
}

inline MotorCurrentVectorMeasurement::MotorCurrentVectorMeasurement(const CanFrame& frame) {
	// Unpack i_q from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t i_q_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&i_q, &i_q_bits, sizeof(float));

	// Unpack i_d from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t i_d_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&i_d, &i_d_bits, sizeof(float));
}

inline MotorBackEMFMeasurement::MotorBackEMFMeasurement(const CanFrame& frame) {
	// Unpack BEMf_q from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t BEMf_q_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&BEMf_q, &BEMf_q_bits, sizeof(float));

	// Unpack BEMf_d from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t BEMf_d_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&BEMf_d, &BEMf_d_bits, sizeof(float));
}

inline VoltageRailMeasurement15V::VoltageRailMeasurement15V(const CanFrame& frame) {
	// Unpack supply_of_15V from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t voltage_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&supply_of_15V, &voltage_bits, sizeof(float));
}

inline VoltageRailMeasurement3V3And1V9::VoltageRailMeasurement3V3And1V9(const CanFrame& frame) {
	// Unpack supply_of_1V9 from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t voltage_1v9_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&supply_of_1V9, &voltage_1v9_bits, sizeof(float));

	// Unpack supply_of_3V3 from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t voltage_3v3_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&supply_of_3V3, &voltage_3v3_bits, sizeof(float));
}

inline HeatsinkAndMotorTemperatureMeasurement::HeatsinkAndMotorTemperatureMeasurement(const CanFrame& frame) {
	// Unpack motor_temp from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t motor_temp_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&motor_temp, &motor_temp_bits, sizeof(float));

	// Unpack heat_sink_temp from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t heat_sink_temp_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&heat_sink_temp, &heat_sink_temp_bits, sizeof(float));
}

inline DSPBoardTemperatureMeasurement::DSPBoardTemperatureMeasurement(const CanFrame& frame) {
	// Unpack DSP_board_temp from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t dsp_temp_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&DSP_board_temp, &dsp_temp_bits, sizeof(float));
}

inline OdometerAndBusAmpHoursMeasurement::OdometerAndBusAmpHoursMeasurement(const CanFrame& frame) {
	// Unpack odometer from first 4 bytes (IEEE 754 float) - MSB first
	uint32_t odometer_bits {
		(static_cast<uint32_t>(frame.data[0]) << 24) |
		(static_cast<uint32_t>(frame.data[1]) << 16) |
		(static_cast<uint32_t>(frame.data[2]) << 8)  |
		(static_cast<uint32_t>(frame.data[3]))
	};

	std::memcpy(&odometer, &odometer_bits, sizeof(float));

	// Unpack DC_bus_amp_hours from next 4 bytes (IEEE 754 float) - MSB first
	uint32_t amp_hours_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&DC_bus_amp_hours, &amp_hours_bits, sizeof(float));
}

inline SlipSpeedMeasurement::SlipSpeedMeasurement(const CanFrame& frame) {
	// Unpack slip_speed from last 4 bytes (IEEE 754 float) - MSB first
	uint32_t slip_speed_bits {
		(static_cast<uint32_t>(frame.data[4]) << 24) |
		(static_cast<uint32_t>(frame.data[5]) << 16) |
		(static_cast<uint32_t>(frame.data[6]) << 8)  |
		(static_cast<uint32_t>(frame.data[7]))
	};

	std::memcpy(&slip_speed, &slip_speed_bits, sizeof(float));
}

inline ActiveMotorChangeCommand::ActiveMotorChangeCommand(const CanFrame& frame) {
	// Unpack access_key from bytes 0-5 (6 ASCII characters) - MSB first
	access_key[5] = frame.data[0];  // 'T'
	access_key[4] = frame.data[1];  // 'O'
	access_key[3] = frame.data[2];  // 'M'
	access_key[2] = frame.data[3];  // 'T'
	access_key[1] = frame.data[4];  // 'C'
	access_key[0] = frame.data[5];  // 'A'

	// Unpack active_motor from bytes 6-7 (uint16_t) - MSB first  
	active_motor = static_cast<uint16_t>(
		(static_cast<uint16_t>(frame.data[6]) << 8) |
		(static_cast<uint16_t>(frame.data[7]))
	);
}

// ============================================================================
// SERIALIZATION FUNCTIONS (Domain Structs -> CAN frames)
// ============================================================================

/// @brief Pack MotorDriveCommand into CAN frame
/// @param cmd The command to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each float
inline CanFrame pack(const MotorDriveCommand& cmd) {
	CanFrame frame{};
	frame.id = ID_MOTOR_DRIVE_COMMAND;
	frame.dlc = 8;  // 4 bytes for velocity + 4 bytes for current percentage

	// Pack motor_velocity_rpm (IEEE 754 32-bit float) - MSB first
	uint32_t velocity_bits;
	std::memcpy(&velocity_bits, &cmd.motor_velocity_rpm, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((velocity_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((velocity_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((velocity_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>(velocity_bits & 0xFF);

	// Pack motor_current_percent (IEEE 754 32-bit float) - MSB first
	uint32_t current_bits;
	std::memcpy(&current_bits, &cmd.motor_current_percent, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((current_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((current_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((current_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>(current_bits & 0xFF);

	return frame;
}

/// @brief Pack MotorPowerCommand into CAN frame
/// @param cmd The command to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each float
inline CanFrame pack(const MotorPowerCommand& cmd) {
	CanFrame frame{};
	frame.id = ID_MOTOR_POWER_COMMAND;
	frame.dlc = 8;  // 4 bytes reserved + 4 bytes for bus current

	// First 4 bytes are reserved (set to zero)
	frame.data[0] = 0;
	frame.data[1] = 0;
	frame.data[2] = 0;
	frame.data[3] = 0;

	// Pack bus_current (IEEE 754 32-bit float) - MSB first
	uint32_t current_bits;
	std::memcpy(&current_bits, &cmd.bus_current, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((current_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((current_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((current_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((current_bits) & 0xFF);

	return frame;
}

/// @brief Pack ResetCommand into CAN frame
/// @param cmd The command to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const ResetCommand& /*cmd*/) {
	CanFrame frame{};
	frame.id = ID_RESET_COMMAND;
	frame.dlc = 0;  // No data for reset command
	return frame;
}

/// @brief Pack IdentificationInformation into CAN frame
/// @param info The information to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const IdentificationInformation& info) {
	CanFrame frame{};
	frame.id = ID_IDENTIFICATION_INFORMATION;
	frame.dlc = 8;  // 4 bytes for serial number + 4 bytes for prohelion ID

	// Pack prohelion_ID - MSB first
	frame.data[0] = static_cast<uint8_t>((info.prohelion_ID >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((info.prohelion_ID >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((info.prohelion_ID >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((info.prohelion_ID) & 0xFF);

	// Pack serial_number - MSB first
	frame.data[4] = static_cast<uint8_t>((info.serial_number >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((info.serial_number >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((info.serial_number >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((info.serial_number) & 0xFF);

	return frame;
}

/// @brief Pack StatusInformation into CAN frame
/// @param info The information to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const StatusInformation& info) {
	CanFrame frame{};
	frame.id = ID_STATUS_INFORMATION;
	frame.dlc = 8;  // 2 bytes limit flags + 2 bytes error flags + 2 bytes active motor + 1 byte tx error count + 1 byte rx error count

	// Pack limit_flags - MSB first
	frame.data[0] = static_cast<uint8_t>((info.limit_flags >> 8) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((info.limit_flags) & 0xFF);

	// Pack error_flags - MSB first
	frame.data[2] = static_cast<uint8_t>((info.error_flags >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((info.error_flags) & 0xFF);

	// Pack active_motor - MSB first
	frame.data[4] = static_cast<uint8_t>((info.active_motor >> 8) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((info.active_motor) & 0xFF);

	// Pack transmit_error_count
	frame.data[6] = info.transmit_error_count;

	// Pack receive_error_count
	frame.data[7] = info.receive_error_count;

	return frame;
}

/// @brief Pack BusMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each float
inline CanFrame pack(const BusMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_BUS_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for voltage + 4 bytes for current

	// Pack bus_voltage (IEEE 754 32-bit float) - MSB first
	uint32_t voltage_bits;
	std::memcpy(&voltage_bits, &measurement.bus_voltage, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((voltage_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((voltage_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((voltage_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((voltage_bits) & 0xFF);

	// Pack bus_current (IEEE 754 32-bit float) - MSB first
	uint32_t current_bits;
	std::memcpy(&current_bits, &measurement.bus_current, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((current_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((current_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((current_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((current_bits) & 0xFF);

	return frame;
}

/// @brief Pack VelocityMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const VelocityMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_VELOCITY_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for motor velocity + 4 bytes for vehicle velocity

	// Pack motor_velocity_rpm (IEEE 754 32-bit float) - MSB first
	uint32_t rpm_bits;
	std::memcpy(&rpm_bits, &measurement.motor_velocity_rpm, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((rpm_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((rpm_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((rpm_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((rpm_bits) & 0xFF);

	// Pack vehicle_velocity (IEEE 754 32-bit float) - MSB first
	uint32_t velocity_bits;
	std::memcpy(&velocity_bits, &measurement.vehicle_velocity, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((velocity_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((velocity_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((velocity_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((velocity_bits) & 0xFF);

	return frame;
}

/// @brief Pack PhaseCurrentMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const PhaseCurrentMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_PHASE_CURRENT_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for phase B current + 4 bytes for phase C current

	// Pack phase_b_current (IEEE 754 32-bit float) - MSB first
	uint32_t phase_b_bits;
	std::memcpy(&phase_b_bits, &measurement.phase_b_current, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((phase_b_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((phase_b_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((phase_b_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((phase_b_bits) & 0xFF);

	// Pack phase_c_current (IEEE 754 32-bit float) - MSB first
	uint32_t phase_c_bits;
	std::memcpy(&phase_c_bits, &measurement.phase_c_current, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((phase_c_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((phase_c_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((phase_c_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((phase_c_bits) & 0xFF);

	return frame;
}

/// @brief Pack MotorVoltageVectorMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const MotorVoltageVectorMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_MOTOR_VOLTAGE_VECTOR_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for v_q + 4 bytes for v_d

	// Pack v_q (IEEE 754 32-bit float) - MSB first
	uint32_t v_q_bits;
	std::memcpy(&v_q_bits, &measurement.v_q, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((v_q_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((v_q_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((v_q_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((v_q_bits) & 0xFF);

	// Pack v_d (IEEE 754 32-bit float) - MSB first
	uint32_t v_d_bits;
	std::memcpy(&v_d_bits, &measurement.v_d, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((v_d_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((v_d_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((v_d_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((v_d_bits) & 0xFF);

	return frame;
}

/// @brief Pack MotorCurrentVectorMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const MotorCurrentVectorMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_MOTOR_CURRENT_VECTOR_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for i_q + 4 bytes for i_d

	// Pack i_q (IEEE 754 32-bit float) - MSB first
	uint32_t i_q_bits;
	std::memcpy(&i_q_bits, &measurement.i_q, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((i_q_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((i_q_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((i_q_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((i_q_bits) & 0xFF);

	// Pack i_d (IEEE 754 32-bit float) - MSB first
	uint32_t i_d_bits;
	std::memcpy(&i_d_bits, &measurement.i_d, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((i_d_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((i_d_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((i_d_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((i_d_bits) & 0xFF);

	return frame;
}

/// @brief Pack MotorBackEMFMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const MotorBackEMFMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_MOTOR_BACK_EMF_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for BEMf_q + 4 bytes for BEMf_d

	// Pack BEMf_q (IEEE 754 32-bit float) - MSB first
	uint32_t BEMf_q_bits;
	std::memcpy(&BEMf_q_bits, &measurement.BEMf_q, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((BEMf_q_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((BEMf_q_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((BEMf_q_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((BEMf_q_bits) & 0xFF);

	// Pack BEMf_d (IEEE 754 32-bit float) - MSB first
	uint32_t BEMf_d_bits;
	std::memcpy(&BEMf_d_bits, &measurement.BEMf_d, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((BEMf_d_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((BEMf_d_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((BEMf_d_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((BEMf_d_bits) & 0xFF);

	return frame;
}

/// @brief Pack VoltageRailMeasurement15V into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const VoltageRailMeasurement15V& measurement) {
	CanFrame frame{};
	frame.id = ID_15V_VOLTAGE_RAIL_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes reserved + 4 bytes for 15V supply

	// First 4 bytes are reserved (set to zero)
	frame.data[0] = 0;
	frame.data[1] = 0;
	frame.data[2] = 0;
	frame.data[3] = 0;

	// Pack supply_of_15V (IEEE 754 32-bit float) - MSB first
	uint32_t voltage_bits;
	std::memcpy(&voltage_bits, &measurement.supply_of_15V, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((voltage_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((voltage_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((voltage_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((voltage_bits) & 0xFF);

	return frame;
}

/// @brief Pack VoltageRailMeasurement3V3And1V9 into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const VoltageRailMeasurement3V3And1V9& measurement) {
	CanFrame frame{};
	frame.id = ID_3V3_AND_1V9_VOLTAGE_RAIL_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for 1.9V supply + 4 bytes for 3.3V supply

	// Pack supply_of_1V9 (IEEE 754 32-bit float) - MSB first
	uint32_t voltage_1v9_bits;
	std::memcpy(&voltage_1v9_bits, &measurement.supply_of_1V9, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((voltage_1v9_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((voltage_1v9_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((voltage_1v9_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((voltage_1v9_bits) & 0xFF);

	// Pack supply_of_3V3 (IEEE 754 32-bit float) - MSB first
	uint32_t voltage_3v3_bits;
	std::memcpy(&voltage_3v3_bits, &measurement.supply_of_3V3, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((voltage_3v3_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((voltage_3v3_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((voltage_3v3_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((voltage_3v3_bits) & 0xFF);

	return frame;
}

/// @brief Pack HeatsinkAndMotorTemperatureMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const HeatsinkAndMotorTemperatureMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_HEATSINK_AND_MOTOR_TEMPERATURE_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for motor temp + 4 bytes for heat sink temp

	// Pack motor_temp (IEEE 754 32-bit float) - MSB first
	uint32_t motor_temp_bits;
	std::memcpy(&motor_temp_bits, &measurement.motor_temp, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((motor_temp_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((motor_temp_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((motor_temp_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((motor_temp_bits) & 0xFF);

	// Pack heat_sink_temp (IEEE 754 32-bit float) - MSB first
	uint32_t heat_sink_temp_bits;
	std::memcpy(&heat_sink_temp_bits, &measurement.heat_sink_temp, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((heat_sink_temp_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((heat_sink_temp_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((heat_sink_temp_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((heat_sink_temp_bits) & 0xFF);

	return frame;
}

/// @brief Pack DSPBoardTemperatureMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const DSPBoardTemperatureMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_DSP_BOARD_TEMPERATURE_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for DSP board temp + 4 bytes reserved

	// Pack DSP_board_temp (IEEE 754 32-bit float) - MSB first
	uint32_t dsp_temp_bits;
	std::memcpy(&dsp_temp_bits, &measurement.DSP_board_temp, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((dsp_temp_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((dsp_temp_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((dsp_temp_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((dsp_temp_bits) & 0xFF);

	// Last 4 bytes are reserved (set to zero)
	frame.data[4] = 0;
	frame.data[5] = 0;
	frame.data[6] = 0;
	frame.data[7] = 0;

	return frame;
}

/// @brief Pack OdometerAndBusAmpHoursMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const OdometerAndBusAmpHoursMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_ODOMETER_AND_BUS_AMP_HOURS_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes for odometer + 4 bytes for DC bus amp hours

	// Pack odometer (IEEE 754 32-bit float) - MSB first
	uint32_t odometer_bits;
	std::memcpy(&odometer_bits, &measurement.odometer, sizeof(float));
	frame.data[0] = static_cast<uint8_t>((odometer_bits >> 24) & 0xFF);
	frame.data[1] = static_cast<uint8_t>((odometer_bits >> 16) & 0xFF);
	frame.data[2] = static_cast<uint8_t>((odometer_bits >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>((odometer_bits) & 0xFF);

	// Pack DC_bus_amp_hours (IEEE 754 32-bit float) - MSB first
	uint32_t amp_hours_bits;
	std::memcpy(&amp_hours_bits, &measurement.DC_bus_amp_hours, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((amp_hours_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((amp_hours_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((amp_hours_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((amp_hours_bits) & 0xFF);

	return frame;
}

/// @brief Pack SlipSpeedMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data low float first then MSB-first byte order within each
inline CanFrame pack(const SlipSpeedMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_SLIP_SPEED_MEASUREMENT;
	frame.dlc = 8;  // 4 bytes reserved + 4 bytes for slip speed

	// First 4 bytes are reserved (set to zero)
	frame.data[0] = 0;
	frame.data[1] = 0;
	frame.data[2] = 0;
	frame.data[3] = 0;

	// Pack slip_speed (IEEE 754 32-bit float) - MSB first
	uint32_t slip_speed_bits;
	std::memcpy(&slip_speed_bits, &measurement.slip_speed, sizeof(float));
	frame.data[4] = static_cast<uint8_t>((slip_speed_bits >> 24) & 0xFF);
	frame.data[5] = static_cast<uint8_t>((slip_speed_bits >> 16) & 0xFF);
	frame.data[6] = static_cast<uint8_t>((slip_speed_bits >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((slip_speed_bits) & 0xFF);

	return frame;
}

/// @brief Pack ActiveMotorChangeCommand into CAN frame
/// @param cmd The command to serialize
/// @return CAN frame with packed data - access key first, then MSB first for motor index
inline CanFrame pack(const ActiveMotorChangeCommand& cmd) {
	CanFrame frame{};
	frame.id = ID_ACTIVE_MOTOR_CHANGE;
	frame.dlc = 8;  // 6 bytes for access_key + 2 bytes for active_motor

	// Pack access_key (6 ASCII bytes) - should be "ACTMOT" - MSB first
	frame.data[0] = cmd.access_key[5];  // 'T'
	frame.data[1] = cmd.access_key[4];  // 'O'
	frame.data[2] = cmd.access_key[3];  // 'M'
	frame.data[3] = cmd.access_key[2];  // 'T'
	frame.data[4] = cmd.access_key[1];  // 'C'
	frame.data[5] = cmd.access_key[0];  // 'A'

	// Pack active_motor (uint16_t) - MSB first
	frame.data[6] = static_cast<uint8_t>((cmd.active_motor >> 8) & 0xFF);
	frame.data[7] = static_cast<uint8_t>((cmd.active_motor) & 0xFF);

	return frame;
}

#endif // PROTOCOL_HPP