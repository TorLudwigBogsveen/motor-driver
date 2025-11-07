#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include "can_frame.hpp"
#include <cstdint>

/// @file protocol.hpp
/// @brief CAN protocol definitions for motor controller communication
/// @note Uses MSB-first (big-endian) byte ordering

// CAN Message Identifiers
constexpr uint32_t ID_DRIVE_COMMAND = 0x100;   ///< Motor Controller Command
constexpr uint32_t ID_BUS_MEASUREMENT = 0x101; ///< Motor Controller Feedback
constexpr uint32_t ID_BMS_STATUS = 0x200;      ///< BMS Status/Monitoring

// Forward Declarations of domain structs
struct DriveCommand;
struct BusMeasurement;

// Forward Declarations of serialization functions
CanFrame pack(const DriveCommand& cmd);
CanFrame pack(const BusMeasurement& measurement);

// ============================================================================
// STRUCT DEFINITIONS
// ============================================================================

/// @brief Drive command message for motor controller
struct DriveCommand {
	int16_t torque_request;  ///< Torque request value
	uint8_t direction;       ///< Direction control

	/// @brief Construct from CAN frame (deserialize)
	explicit DriveCommand(const CanFrame& frame);

	/// @brief Default constructor
	DriveCommand() : torque_request{0}, direction{0} {}
};

/// @brief Bus measurement message from motor controller
struct BusMeasurement {
	uint16_t voltage_mV;  ///< Bus voltage in millivolts
	int16_t current_mA;   ///< Bus current in milliamps

	/// @brief Construct from CAN frame (deserialize)
	explicit BusMeasurement(const CanFrame& frame);

	/// @brief Default constructor
	BusMeasurement() : voltage_mV{0}, current_mA{0} {}
};

// ============================================================================
// DESERIALIZATION IMPLEMENTATIONS
// ============================================================================
// (unpack CAN frames into domain structs)

inline DriveCommand::DriveCommand(const CanFrame& frame) {
	// Unpack torque_request (int16_t) - MSB first
	torque_request = static_cast<int16_t>(
		(static_cast<uint16_t>(frame.data[0]) << 8) |
		static_cast<uint16_t>(frame.data[1])
		);

	// Unpack direction (uint8_t)
	direction = frame.data[2];
}

inline BusMeasurement::BusMeasurement(const CanFrame& frame) {
	// Unpack voltage_mV (uint16_t) - MSB first
	voltage_mV = static_cast<uint16_t>(
		(static_cast<uint16_t>(frame.data[0]) << 8) |
		static_cast<uint16_t>(frame.data[1])
		);

	// Unpack current_mA (int16_t) - MSB first
	current_mA = static_cast<int16_t>(
		(static_cast<uint16_t>(frame.data[2]) << 8) |
		static_cast<uint16_t>(frame.data[3])
		);
}

// ============================================================================
// SERIALIZATION FUNCTIONS
// ============================================================================
// (pack domain structs into CAN frames)

/// @brief Pack DriveCommand into CAN frame
/// @param cmd The command to serialize
/// @return CAN frame with packed data (MSB-first byte order)
inline CanFrame pack(const DriveCommand& cmd) {
	CanFrame frame{};
	frame.id = ID_DRIVE_COMMAND;
	frame.dlc = 3;

	// Pack torque_request (int16_t) - MSB first
	frame.data[0] = static_cast<uint8_t>((cmd.torque_request >> 8) & 0xFF);
	frame.data[1] = static_cast<uint8_t>(cmd.torque_request & 0xFF);

	// Pack direction (uint8_t)
	frame.data[2] = cmd.direction;

	return frame;
}

/// @brief Pack BusMeasurement into CAN frame
/// @param measurement The measurement to serialize
/// @return CAN frame with packed data (MSB-first byte order)
inline CanFrame pack(const BusMeasurement& measurement) {
	CanFrame frame{};
	frame.id = ID_BUS_MEASUREMENT;
	frame.dlc = 4;

	// Pack voltage_mV (uint16_t) - MSB first
	frame.data[0] = static_cast<uint8_t>((measurement.voltage_mV >> 8) & 0xFF);
	frame.data[1] = static_cast<uint8_t>(measurement.voltage_mV & 0xFF);

	// Pack current_mA (int16_t) - MSB first
	frame.data[2] = static_cast<uint8_t>((measurement.current_mA >> 8) & 0xFF);
	frame.data[3] = static_cast<uint8_t>(measurement.current_mA & 0xFF);

	return frame;
}

#endif // PROTOCOL_HPP