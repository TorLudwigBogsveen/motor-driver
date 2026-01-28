#ifndef WAVESCULPTOR_SIMULATOR_HPP
#define WAVESCULPTOR_SIMULATOR_HPP

#include "can/can_protocol.hpp"
#include "clock.hpp"
#include <cstdint>

/// @brief Motor controller simulator class
class MotorControllerSimulator
{
private:
    struct MotorControllerSimulator::SimulationState
    {
        // Identification
        uint32_t serial_number{0x00001A4F};
        uint32_t prohelion_ID{0x00004003};

        // Status information
        uint8_t receive_error_count{0};
        uint8_t transmit_error_count{0};
        uint16_t active_motor{0};
        uint16_t error_flags{0};
        uint16_t limit_flags{0};

        // Bus measurements
        float bus_current_a{20.5f};
        float bus_voltage_v{100.0f};

        // Velocity measurements
        float vehicle_velocity_m_s{0.0f};
        float motor_velocity_rpm{0.0f};

        // Phase current measurement
        float motor_phase_c_current{32.1f};
        float motor_phase_b_current{32.1f};

        // Motor voltage and current vectors
        float motor_v_d{0.0f}, motor_v_q{65.0f};
        float motor_i_d{-0.5f}, motor_i_q{35.0f};

        // BackEMF
        float bemf_d{0.0f}, bemf_q{80.5f};

        // Supply voltages
        float supply_of_15v{14.9f}, supply_of_3v3{3.31f}, supply_of_1v9{1.91f};

        // Temperatures
        float heat_sink_temperature_c{45.0f};
        float motor_temperature_c{60.0f};
        float dsp_board_temperature_c{35.5f};

        // Odometer & Amp-hours
        float dc_bus_amp_hours{105.7f};
        float odometer_m{185000.0f};

        // Command targets
        float target_motor_current_percent{0.0f};
        float target_motor_velocity_rpm{0.0f};
        float power_command_bus_current_limit_percent{1.0f};

        // Last command times
        uint32_t last_drive_command_time{0};
        uint32_t last_power_command_time{0};
        uint32_t last_simulation_update_time{0};

        // Last periodic message times
        uint32_t last_identification_time{0};
        uint32_t last_status_info_time{0};
        uint32_t last_bus_measurement_time{0};
        uint32_t last_velocity_measurement_time{0};
        uint32_t last_phase_current_measurement_time{0};
        uint32_t last_motor_voltage_vector_measurement_time{0};
        uint32_t last_motor_current_vector_measurement_time{0};
        uint32_t last_motor_backemf_time{0};
        uint32_t last_15v_rail_time{0};
        uint32_t last_3v3_1v9_rail_time{0};
        uint32_t last_heatsink_motor_temp_time{0};
        uint32_t last_dsp_board_temp_time{0};
        uint32_t last_odometer_amp_hours_time{0};
    };

    SimulationState state;

public:
    /// @brief Process received CAN commands (Drive, Power, Reset)
    void processIncomingCommand(const CanFrame &frame);

    /// @brief Send periodic status and measurement messages
    void sendPeriodicMessages(uint32_t current_time_ms, void* t_state, void(*transmit_func)(const CanFrame&, void*));

    /// @brief Update simulation physics and state
    void updateSimulation(uint32_t current_time_ms);
};

#endif // WAVESCULPTOR_SIMULATOR_HPP