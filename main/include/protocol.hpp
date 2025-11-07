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
constexpr uint32_t DRIVER_CONTROLS_BASE_ADDRESS = 0x500; ///< Base address for driver control commands
constexpr uint32_t MOTOR_CONTROLLER_BASE_ADDRESS = 0x400; ///< Base address for motor controller messages

// CAN Message Identifiers for Drive Commands
constexpr uint32_t ID_MOTOR_DRIVE_COMMAND = DRIVER_CONTROLS_BASE_ADDRESS + 0x01;   ///< Motor Controller Command, WaveSculptor22 motor controller must receive a Motor Drive Command frame at least once every 250ms!
constexpr uint32_t ID_MOTOR_POWER_COMMAND = DRIVER_CONTROLS_BASE_ADDRESS + 0x02;   ///< Motor Power Command
constexpr uint32_t ID_RESET_COMMAND = DRIVER_CONTROLS_BASE_ADDRESS + 0x03;         ///< System Reset Command

// CAN Message Identifiers for Motor Controll Broadcast Messages
constexpr uint32_t ID_IDENTIFICATION_INFORMATION = MOTOR_CONTROLLER_BASE_ADDRESS + 0x00; ///< Identification Information (Interval 1000ms)
constexpr uint32_t ID_STATUS_INFORMATION = MOTOR_CONTROLLER_BASE_ADDRESS + 0x01;         ///< Status Information (Interval 200ms)
constexpr uint32_t ID_BUS_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x02; ///< Bus Measurement (Interval 200ms)
constexpr uint32_t ID_VELOCITY_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x03; ///< Velocity Measurement (Interval 200ms)
constexpr uint32_t ID_PHASE_CURRENT_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x04; ///< Phase Current Measurement (Interval 200ms)
constexpr uint32_t ID_MOTOR_VOLTAGE_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x05; ///< Motor Voltage Measurement (Interval 200ms)
constexpr uint32_t ID_MOTOR_CURRENT_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x06; ///< Motor Current Measurement (Interval 200ms)
constexpr uint32_t ID_MOTORBACKEMF_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x07; ///< Motor Back-EMF Measurement/Prediction (Interval 200ms)
constexpr uint32_t ID_15V_VOLTAGE_RAIL_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x08; ///< 15V Voltage Rail Measurement (Interval 1000ms)
constexpr uint32_t ID_3V3_AND_19V_VOLTAGE_RAIL_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x09; ///< 3.3V and 19V Voltage Rail Measurement (Interval 1000ms)
// ID: MOTOR_CONOTROLLER_BASE_ADDRESS + 0x0A is reserved
constexpr uint32_t ID_HEATSINK_AND_MOTOR_TEMPERATURE_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x0B; ///< Heatsink and Motor Temperature Measurement (Interval 1000ms)
constexpr uint32_t ID_DSP_BOARD_TEMPERATURE_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x0C; ///< DSP Board Temperature Measurement (Interval 1000ms)
// ID: MOTOR_CONTROLLER_BASE_ADDRESS + 0x0D is reserved
constexpr uint32_t ID_ODOMETER_AND_BUS_AMP_HOURS_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x0E; ///< Odometer and Bus Amp-Hours Measurement (Interval 1000ms)
constexpr uint32_t ID_SLIP_SPEED_MEASUREMENT = MOTOR_CONTROLLER_BASE_ADDRESS + 0x17; ///< Slip Speed Measurement (Interval 200ms)

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
struct MotorVoltageMeasurement;
struct MotorCurrentMeasurement;
struct MotorBackEMFMeasurement;
struct VoltageRailMeasurement15V;
struct VoltageRailMeasurement3V3And19V;
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
CanFrame pack(const MotorVoltageMeasurement& measurement);
CanFrame pack(const MotorCurrentMeasurement& measurement);
CanFrame pack(const MotorBackEMFMeasurement& measurement);
CanFrame pack(const VoltageRailMeasurement15V& measurement);
CanFrame pack(const VoltageRailMeasurement3V3And19V& measurement);
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

/// @brief Bus measurement message from motor controller
struct BusMeasurement {
	float bus_current;  ///< Bus current in amps
	float bus_voltage;   ///< Bus voltage in volts

	/// @brief Construct from CAN frame (deserialize)
	explicit BusMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	BusMeasurement() : bus_current{0}, bus_voltage{0} {}
};

// ============================================================================
// DESERIALIZATION IMPLEMENTATIONS
// ============================================================================
// (unpack CAN frames into domain structs)

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

// ============================================================================
// SERIALIZATION FUNCTIONS
// ============================================================================
// (pack domain structs into CAN frames)

/// @brief Pack MotorDriveCommand into CAN frame
/// @param cmd The command to serialize
/// @return CAN frame with packed data (MSB-first byte order)
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

/// @brief Pack BusMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data (MSB-first byte order)
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

#endif // PROTOCOL_HPP