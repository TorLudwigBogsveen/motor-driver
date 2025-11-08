/// @mainpage CAN Motor Controller Simulator
/// 
/// @section intro_sec Introduction
/// 
/// This project implements a comprehensive CAN protocol for communicating with 
/// WaveSculptor motor controllers on ESP32 microcontrollers. It provides type-safe, 
/// well-documented C++ structs for all motor control messages with IEEE 754 float 
/// handling and MSB-first byte ordering.
/// 
/// @section features_sec 🌟 Key Features
/// 
/// - ✅ **Complete WaveSculptor Protocol Implementation** - All message types supported
/// - ✅ **Type-Safe IEEE 754 Float Handling** - Proper serialization/deserialization  
/// - ✅ **MSB-First Byte Ordering** - Consistent with CAN protocol standards
/// - ✅ **Comprehensive Documentation** - Detailed error and limit flag descriptions
/// - ✅ **Platform Compatibility Checks** - Static assertions prevent runtime failures
/// - ✅ **Modern C++ Design** - Zero-cost abstractions with clean APIs
/// - ✅ **Embedded-Friendly** - No dynamic allocation, predictable performance
/// 
/// @section architecture_sec 🏗️ Architecture
/// 
/// The system is organized into several key components:
/// 
/// ### Core Protocol Layer
/// - **Domain Structs**: Type-safe representations of CAN messages
/// - **Serialization Functions**: Convert structs to/from CAN frames
/// - **Message Identifiers**: Standardized CAN IDs for all message types
/// 
/// ### Hardware Abstraction
/// - **CAN Driver**: ESP32 TWAI peripheral abstraction
/// - **Frame Structures**: Low-level CAN frame representations
/// 
/// @section usage_sec 🚀 Quick Start
/// 
/// ### Basic Motor Control
/// @code
/// #include "protocol.hpp"
/// #include "can_driver.hpp"
/// 
/// // Create motor command
/// MotorDriveCommand cmd{};
/// cmd.motor_current_percent = 0.75f;  // 75% current
/// cmd.motor_velocity_rpm = 1800.0f;   // 1800 RPM forward
/// 
/// // Send command
/// CanFrame frame = pack(cmd);
/// can_driver.transmit(frame);
/// 
/// // Receive status
/// CanFrame status_frame;
/// if (can_driver.receive(status_frame)) {
///     if (status_frame.id == ID_STATUS_INFORMATION) {
///         StatusInformation status{status_frame};
///         
///         // Check for errors
///         if (status.error_flags & 0x0001) {
///             printf("Hardware overcurrent detected!\n");
///         }
///     }
/// }
/// @endcode
/// 
/// ### Motor Configuration
/// @code
/// // Change active motor (use sparingly - saves to EEPROM!)
/// ActiveMotorChangeCommand motor_change{5};  // Switch to motor 5
/// CanFrame change_frame = pack(motor_change);
/// can_driver.transmit(change_frame);
/// @endcode
/// 
/// ### Reading Measurements
/// @code
/// // Process bus measurements
/// CanFrame bus_frame;
/// if (can_driver.receive(bus_frame) && bus_frame.id == ID_BUS_MEASUREMENT) {
///     BusMeasurement measurement{bus_frame};
///     printf("Bus: %.1fV, %.2fA\n", measurement.bus_voltage, measurement.bus_current);
/// }
/// @endcode
/// 
/// @section messages_sec 📡 Message Types
/// 
/// ### Control Commands
/// - **MotorDriveCommand**: Primary motor control (velocity + current)
/// - **MotorPowerCommand**: Power-based control
/// - **ResetCommand**: Software reset
/// - **ActiveMotorChangeCommand**: Switch active motor
/// 
/// ### Status & Measurements  
/// - **StatusInformation**: Error flags, limits, CAN counters
/// - **BusMeasurement**: Voltage and current measurements
/// - **VelocityMeasurement**: Speed and position data
/// - **TemperatureMeasurement**: Thermal monitoring
/// - **IdentificationInformation**: Device identification
/// 
/// @section safety_sec ⚠️ Safety Considerations
/// 
/// ### Critical Timing Requirements
/// - **Motor Drive Commands**: Must be sent at least once every 250ms or motor stops
/// - **CAN Bus Termination**: Requires 120Ω termination resistors
/// - **EEPROM Wear**: Avoid frequent ActiveMotorChangeCommand usage
/// 
/// ### Error Handling
/// - Monitor `error_flags` in StatusInformation messages
/// - Implement proper timeout handling for command acknowledgments
/// - Use `limit_flags` to understand motor controller limitations
/// 
/// @section hardware_sec 🔧 Hardware Configuration
/// 
/// ### ESP32 CAN Setup
/// - **CAN TX**: GPIO21
/// - **CAN RX**: GPIO22  
/// - **Baud Rate**: 500 kbps
/// - **Termination**: 120Ω resistors required on both ends
/// 
/// ### WaveSculptor Motor Controller
/// - **Base Address**: 0x400 (configurable)
/// - **Driver Controls**: 0x500 (configurable)
/// - **Protocol**: MSB-first byte ordering
/// - **Timing**: Various message intervals (200ms - 1000ms)
/// 
/// @section technical_sec 🔬 Technical Details
/// 
/// ### IEEE 754 Float Handling
/// The protocol uses 32-bit IEEE 754 floats with MSB-first byte ordering:
/// 
/// @code
/// // Serialization example
/// float value = 123.45f;
/// uint32_t bits;
/// std::memcpy(&bits, &value, sizeof(float));
/// 
/// // Pack as MSB-first
/// frame.data[0] = static_cast<uint8_t>((bits >> 24) & 0xFF);
/// frame.data[1] = static_cast<uint8_t>((bits >> 16) & 0xFF);
/// frame.data[2] = static_cast<uint8_t>((bits >> 8) & 0xFF);
/// frame.data[3] = static_cast<uint8_t>(bits & 0xFF);
/// @endcode
/// 
/// ### Platform Compatibility
/// Static assertions ensure correct platform support:
/// 
/// @code
/// static_assert(sizeof(float) == 4, "Platform must use 32-bit float");
/// static_assert(std::numeric_limits<float>::is_iec559, "Platform must use IEEE 754");
/// @endcode
/// 
/// @section performance_sec ⚡ Performance Characteristics
/// 
/// ### Memory Usage
/// - **Zero Dynamic Allocation**: All structs use fixed-size members
/// - **Minimal RAM Footprint**: Efficient struct packing
/// - **Stack-Based Objects**: Fast allocation/deallocation
/// 
/// ### Execution Time
/// - **Predictable Serialization**: Fixed number of operations
/// - **Cache-Friendly**: Sequential memory access patterns
/// - **Branch-Free Bit Operations**: Optimal for real-time systems
/// 
/// @section contributing_sec 🤝 Contributing
/// 
/// When adding new message types:
/// 1. Add CAN message identifier constant
/// 2. Create domain struct with proper documentation
/// 3. Implement constructor for deserialization  
/// 4. Implement pack() function for serialization
/// 5. Add comprehensive bit-field documentation for flags
/// 
/// @section license_sec 📄 License
/// 
/// This project is part of the CAN Motor Controller Simulation System.
/// 
/// @author Your Name
/// @date November 2025
/// @version 1.0