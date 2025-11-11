# 🚗⚡ WaveSculptor Motor Controller Simulator

A comprehensive ESP32-based simulation of a Prohelion WaveSculptor motor controller for solar car development and testing.

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4.3-blue.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-orange.svg)](https://en.cppreference.com/w/cpp/17)
[![CAN Bus](https://img.shields.io/badge/CAN-Bus-green.svg)](https://en.wikipedia.org/wiki/CAN_bus)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [CAN Protocol](#can-protocol)
- [Simulation Physics](#simulation-physics)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Documentation](#documentation)
- [Contributing](#contributing)

## 🎯 Overview

This project simulates a **Prohelion WaveSculptor motor controller** on an ESP32 microcontroller, providing realistic motor control behavior for solar car development. The simulator implements the complete WaveSculptor CAN protocol with authentic motor physics, thermal modeling, and safety systems.

### 🏁 **Perfect for Solar Car Teams:**
- **Protocol Testing**: Validate ECU communication without real motor controller
- **Development Environment**: Test control algorithms in a safe, controlled environment
- **Educational Tool**: Learn motor control principles and CAN bus communication
- **System Integration**: Test complete vehicle systems before hardware deployment

## ✨ Features

### 🔧 **Motor Controller Simulation:**
- ✅ **Realistic Motor Physics**: Acceleration dynamics, torque curves, speed control
- ✅ **Thermal Modeling**: Motor and heatsink temperature simulation with Newton's Law of Cooling
- ✅ **Power Management**: Global power limiting with configurable current caps
- ✅ **Safety Systems**: Command timeouts, over-temperature protection, current limiting

### 📡 **CAN Communication:**
- ✅ **Complete WaveSculptor Protocol**: All message types supported
- ✅ **IEEE 754 Compliance**: Proper float serialization/deserialization
- ✅ **Real-time Messaging**: Configurable transmission intervals (200ms-1000ms)
- ✅ **Error Handling**: Proper error and limit flag management

### ⚙️ **Advanced Features:**
- ✅ **Delta-time Physics**: Frame-rate independent simulation
- ✅ **Command Processing**: Drive, Power, and Reset command support
- ✅ **Comprehensive Logging**: ESP-IDF logging with rate limiting
- ✅ **Professional Documentation**: Doxygen-ready code documentation

## 🔌 Hardware Requirements

### **ESP32 Development Board**
- **Microcontroller**: ESP32 (dual-core, 240 MHz)
- **Flash Memory**: Minimum 4MB
- **RAM**: Minimum 520KB

### **CAN Transceiver Module**
- **IC**: MCP2515 or SN65HVD230
- **Interface**: SPI (for MCP2515) or direct (for SN65HVD230)
- **Voltage**: 3.3V or 5V compatible

### **Optional Components**
- **Termination Resistors**: 120Ω (if at bus endpoints)
- **Power Supply**: 5V/3.3V depending on transceiver
- **Status LEDs**: For visual feedback

## 💻 Software Requirements

### **Development Environment**
```bash
# ESP-IDF Framework
ESP-IDF v5.4.3 or later

# Build Tools
CMake 3.16 or later
Python 3.8 or later

# Development IDE (recommended)
VS Code with ESP-IDF extension
```

### **Dependencies**
- **FreeRTOS**: Real-time task management
- **ESP-IDF CAN Driver**: TWAI (Two-Wire Automotive Interface)
- **Standard C++ Libraries**: `<cmath>`, `<algorithm>`, `<stdio.h>`

## 🚀 Installation

### **1. Clone Repository**
```bash
git clone https://github.com/KrapnixFootball/can-simulation-system.git
cd can-simulation-system/CAN_MotorController_Simulator
```

### **2. Setup ESP-IDF Environment**
```bash
# Install ESP-IDF (if not already installed)
# Follow: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

# Set up environment
. $HOME/esp/esp-idf/export.sh
```

### **3. Configure Project**
```bash
# Open configuration menu
idf.py menuconfig

# Configure:
# - Serial port settings
# - CAN bus settings (GPIO pins, baud rate)
# - WiFi settings (if needed)
```

### **4. Build and Flash**
```bash
# Build project
idf.py build

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash monitor

# For Windows:
# idf.py -p COM3 flash monitor
```

## 📖 Usage

### **Basic Operation**

1. **Power On**: ESP32 boots and initializes CAN driver
2. **CAN Communication**: Simulator begins transmitting periodic messages
3. **Command Processing**: Responds to Drive, Power, and Reset commands
4. **Real-time Simulation**: Updates motor physics at 100Hz (10ms intervals)

### **CAN Bus Setup**

```cpp
// Default CAN Configuration
CAN Baud Rate: 500 kbps
GPIO TX Pin: 21
GPIO RX Pin: 22
Termination: 120Ω (if required)
```

### **Command Examples**

```cpp
// Drive Command (ID: 0x501)
MotorDriveCommand drive_cmd;
drive_cmd.motor_velocity_rpm = 3500.0f;    // Target RPM
drive_cmd.motor_current_percent = 0.8f;    // 80% torque (0.8 = 80%)

// Power Command (ID: 0x502) 
MotorPowerCommand power_cmd;
power_cmd.bus_current = 0.5f;              // 50% power limit (0.5 = 50%)

// Reset Command (ID: 0x503)
ResetCommand reset_cmd;
// Resets simulation to default state
```

## 📡 CAN Protocol

### **Transmitted Messages (Simulator → ECU)**

| Message Type | ID | Frequency | Description |
|--------------|----|-----------| ------------|
| **Status Information** | 0x40E | 200ms | Error flags, limit flags, motor status |
| **Bus Measurement** | 0x403 | 200ms | Bus voltage, bus current |
| **Velocity Measurement** | 0x402 | 200ms | Motor RPM, vehicle velocity |
| **Phase Current** | 0x404 | 200ms | 3-phase motor currents |
| **Motor Voltage Vector** | 0x405 | 200ms | d-q axis voltages |
| **Motor Current Vector** | 0x406 | 200ms | d-q axis currents |
| **BackEMF Measurement** | 0x407 | 200ms | d-q axis back-EMF |
| **Temperature Measurement** | 0x40B | 1000ms | Motor and heatsink temperatures |
| **Identification** | 0x400 | 1000ms | Serial number, device ID |

### **Received Commands (ECU → Simulator)**

| Command Type | ID | Description |
|--------------|----| ------------|
| **Drive Command** | 0x501 | Motor velocity and current targets |
| **Power Command** | 0x502 | Global power limiting percentage |
| **Reset Command** | 0x503 | Reset simulation state |

## ⚙️ Simulation Physics

### **Motor Dynamics**
```cpp
// Current Calculation
Total Current = Base Current + Speed Losses + Torque Demand
Base Current = 2.5A                    // Electronics overhead
Speed Losses = RPM × 0.005A/RPM        // Aerodynamic + friction
Torque Current = Command × 50A          // Maximum torque current

// Acceleration Limiting  
Max Acceleration = 1000 RPM/s          // Realistic motor dynamics
Velocity Change = Acceleration × ΔTime  // Delta-time integration
```

### **Thermal Model**
```cpp
// Newton's Law of Cooling
Heat Generation = I²R                   // Electrical losses (I²R heating)
Cooling Rate = (T_motor - T_ambient) × 0.02  // Convective cooling
Temperature Change = (Heat_Gen - Cool_Rate) × ΔTime
```

### **Power Limiting**
```cpp
// Global Power Management
Max Current = 100A                      // Hardware limit
Power Limited = Max × Power_Command     // ECU-controlled limit  
Actual Current = min(Requested, Power_Limited)  // Apply limiting
```

## 📁 Project Structure

```
CAN_MotorController_Simulator/
├── main/
│   ├── main.cpp                 # Main simulation logic
│   ├── CMakeLists.txt          # Build configuration
│   └── include/
│       ├── protocol.hpp        # CAN protocol implementation
│       └── can_driver.hpp      # CAN driver interface
├── docs/
│   ├── html/                   # Generated Doxygen documentation
│   ├── Doxyfile.custom         # Doxygen configuration
│   ├── custom_light_theme.css  # Documentation styling
│   └── generate_docs.bat       # Documentation build script
├── CMakeLists.txt              # Top-level build config
├── README.md                   # This file
└── sdkconfig                   # ESP-IDF configuration
```

## ⚙️ Configuration

### **Motor Parameters**
```cpp
// Edit in main.cpp - SimulationState struct
float bus_voltage_v{100.0f};           // Battery voltage
float motor_velocity_rpm{3500.0f};     // Cruising RPM
float wheel_radius_m{0.262f};          // Wheel radius
float max_allowed_current{100.0f};     // Current limit
```

### **CAN Settings**
```cpp  
// Edit in can_driver.hpp
#define CAN_BAUD_RATE 500000           // 500 kbps
#define CAN_TX_PIN 21                  // GPIO pin
#define CAN_RX_PIN 22                  // GPIO pin
```

### **Timing Configuration**
```cpp
// Message intervals (in ms)
Status Information:     200ms
Bus Measurements:       200ms  
Velocity Measurements:  200ms
Temperature Data:       1000ms
Identification:         1000ms
```

## 📚 Documentation

### **Generate HTML Documentation**
```bash
# Install Doxygen
# Ubuntu: sudo apt install doxygen
# Windows: Download from doxygen.org

# Generate documentation
cd docs
doxygen Doxyfile.custom

# Open documentation
# Windows: start html/index.html
# Linux: xdg-open html/index.html
```

### **Code Documentation**
- **Comprehensive Comments**: Every function and class documented
- **Doxygen Compatible**: Professional HTML documentation generation
- **Physics Explanations**: Mathematical models explained in comments
- **Protocol References**: Links to WaveSculptor documentation

## 🤝 Contributing

We welcome contributions! Please follow these guidelines:

### **Code Style**
- **C++ Standards**: Use modern C++17 features
- **Naming**: `snake_case` for variables, `PascalCase` for classes
- **Comments**: Document complex physics and protocol implementations
- **Formatting**: Consistent indentation and spacing

### **Submission Process**
1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### **Testing**
- Test with real CAN hardware when possible
- Verify protocol compliance with WaveSculptor documentation
- Check thermal and motor physics for realism

## 🙏 Acknowledgments

- **Prohelion**: WaveSculptor motor controller documentation and protocols
- **Espressif Systems**: ESP-IDF framework and excellent documentation
- **Solar Car Community**: Inspiration and testing feedback
- **Contributors**: Everyone who helped improve this simulation

---

**🚗 Happy Solar Car Development! ⚡**

*Built with ❤️ for the solar racing community*
