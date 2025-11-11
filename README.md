# 🚗⚡ WaveSculptor Motor Controller Simulator

An ESP32 based simulation of a Prohelion WaveSculptor motor controller for solar car development and testing.

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4.3-blue.svg)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-orange.svg)](https://en.cppreference.com/w/cpp/17)
[![CAN Bus](https://img.shields.io/badge/CAN-Bus-green.svg)](https://en.wikipedia.org/wiki/CAN_bus)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## 📋 Table of Contents

- [Overview](#overview)
- [What are some features?](#what-are-some-features)
- [So what hardware do I need?](#so-what-hardware-do-i-need)
- [Software Requirements](#software-requirements)
- [Usage](#usage)
- [CAN Protocol](#can-protocol)
- [Configuration](#configuration)
- [Acknowledgments](#acknowledgments)

## Overview

This project simulates a **Prohelion WaveSculptor motor controller** on an ESP32 microcontroller, providing motor control behavior for my university's solar car development. The simulator implements the complete WaveSculptor22 CAN protocol as it is written in their documentation with pretty okay motor physics, thermal modeling, and safety systems.

### The goal with this project:**
- **Protocol Testing**: Validate ECU communication without real motor controller. This is in consideration to the fact that our solar car is located in a different campus. 
- **Development Environment**: Test control algorithms in a safe and more controlled environment. 
- **Educational Tool**: Learn motor control principles and CAN bus communication. If anyone in the future joins our team and is still beginning to understand CAN, then it is a perfect tool for them.

## What are some features?

### **Motor Controller Simulation:**
- **Realistic Motor Physics**: Acceleration dynamics, torque curves, speed control
- **Thermal Modeling**: Motor and heatsink temperature simulation with Newton's Law of Cooling
- **Power Management**: Global power limiting with configurable current caps
- **Safety Systems**: Command timeouts, over temperature protection, current limiting just like it should be

### **CAN Communication:**
- **Complete WaveSculptor Protocol**: All message types supported
- **IEEE 754 Compliance**: Proper float serialization/deserialization

### **Worth mentioning:**
- **Delta time Physics**: Frame-rate independent simulation
- **Command Processing**: Drive, Power, and Reset command support (it doesn't just send messages but it can adapt according to messages sent to the simulator)
- **Logging**: Basic ESP-IDF logging with rate limiting

## So what hardware do I need?

### **ESP32 Development Board**
- **Microcontroller**: ESP32
- **Flash Memory**: Minimum 4MB
- **RAM**: Minimum 520KB

### **CAN Transceiver Module**
- **IC**: MCP2515 or SN65HVD230 or something of the liking (as ESP32 has no CAN transceiver built in)
- **Voltage**: 3.3V

## Software Requirements

### **Development Environment**
```bash
# ESP-IDF Framework
ESP-IDF v5.4.3 or later

# Build Tools
CMake 3.16 or later
Python 3.8 or later

# Development IDE that I used
VS Code with ESP-IDF extension
```

### **Dependencies**
- **FreeRTOS**: Real-time task management
- **ESP-IDF CAN Driver**: TWAI (Two-Wire Automotive Interface)
- **Standard C++ Libraries**: `<cmath>`, `<algorithm>`, `<stdio.h>`



## Usage

### Build and Flash**
```bash
# Build project
idf.py build

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash monitor

# For Windows:
# idf.py -p COM3 flash monitor
```

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

## CAN Protocol

### **Transmitted Messages**

Exactly the same as is specified on the WaveSculptor22 documentation!

## Configuration

### **Motor Parameters**
```cpp
// Edit in main.cpp - SimulationState struct
float bus_voltage_v{100.0f};           // Battery voltage
float motor_velocity_rpm{3500.0f};     // Cruising RPM
float wheel_radius_m{0.262f};          // Wheel radius
float max_allowed_current{100.0f};     // Current limit
```

## Acknowledgments

- **Prohelion**: WaveSculptor motor controller documentation and protocols
- **Espressif Systems**: ESP-IDF framework and excellent documentation
---