#include "can_driver.hpp"
#include "protocol.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include <stdio.h>
#include <cmath>       // For sin, abs functions
#include <algorithm>   // For std::min

static const char* TAG = "MOTOR_SIM";

/// @brief Motor controller simulation state and behavior
class MotorControllerSimulator {
private:
    // Realistic simulation parameters (cruising state at 55km/h)
    struct SimulationState {
        
        // Identification
        uint32_t serial_number{ 0x00001A4F }; // Example serial number (don't actually know)
        uint32_t prohelion_ID{ 0x00004003 }; // Example ProHelion ID (from the docs)

        // Status information (System state)
        uint8_t receive_error_count{0}; 
        uint8_t transmit_error_count{0};
        uint16_t active_motor{0};
        uint16_t error_flags{0};
        uint16_t limit_flags{0};

        // Bus Measurements
        float bus_current_a{20.5f};
        float bus_voltage_v{100.0f}; // Could change later to our specific one

        // Velocity Measurements
        float vehicle_velocity_m_s{15.28f}; // 55 km/h, calculated from 3500 RPM and wheel size of 0.2619047619m radius
        float motor_velocity_rpm{3500.0f}; // Corresponding realistic motor RPM at 55 km/h

        // Phase Current Measurement
        float motor_phase_c_current{32.1f}; // RMS phase current is typically higher than the DC bus current due the PWM
        float motor_phase_b_current{32.1f}; // (Corrected typo in docs) It should be equal to phase C current at speed

        // Motor Voltage Vector Measurement (UNSURE OF THIS)
        float motor_v_d{0.0f};
        float motor_v_q{65.0f};

        // Motor Current Vector Measurement
        float motor_i_d{-0.5f};
        float motor_i_q{35.0f};

        // Motor BackEMF Measurement
        float bemf_d{0.0f};
        float bemf_q{80.5f};

        // 15V Voltage Rail Measurement
        float supply_of_15v{14.9f};

        // 3V3 and 1V9 Voltage Rail Measurement
        float supply_of_3v3{3.31f};
        float supply_of_1v9{1.91f};

        // Heat Sink and Motor Temperature Measurement
        float heat_sink_temperature_c{45.0f};
        float motor_temperature_c{60.0f};

        // DSP Board Temperature Measurement
        float dsp_board_temperature_c{35.5f};

        // Odometer and Bus Amp-Hours Measurement
        float dc_bus_amp_hours{105.7f};
        float odometer_m{185000.0f}; // 185 km

        // Command targets and actual values
        float target_motor_current_percent{0.0f};
        float target_motor_velocity_rpm{0.0f};
        float power_command_bus_current_limit_percent{1.0f};  // Power limiter (1.0 = 100%, default: no limit)
        
        uint32_t last_drive_command_time{0};  // Track when last command was received
        uint32_t last_power_command_time{0};  // Track when last power command was received
        uint32_t last_simulation_update_time{0};  // For delta-time calculations
        
        // Timing for periodic messages (in ms)
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

    } state;

public:
    /// @brief Process received CAN commands (Drive, Power, Reset)
    void processIncomingCommand(const CanFrame& frame);
    
    /// @brief Send periodic status and measurement messages  
    void sendPeriodicMessages(uint32_t current_time_ms, CanDriver& can_driver);
    
    /// @brief Update simulation physics and state
    void updateSimulation(uint32_t current_time_ms);
};

// Implementation of MotorControllerSimulator methods
void MotorControllerSimulator::processIncomingCommand(const CanFrame& frame) {
    // Handle Drive Commands
    if (frame.id == ID_MOTOR_DRIVE_COMMAND) {
        MotorDriveCommand cmd{frame};
        state.target_motor_velocity_rpm = cmd.motor_velocity_rpm;
        state.target_motor_current_percent = cmd.motor_current_percent;
        
        state.last_drive_command_time = pdTICKS_TO_MS(xTaskGetTickCount());
        ESP_LOGI(TAG, "Drive Command: %.1f%% current, %.0f RPM", 
                 cmd.motor_current_percent * 100.0f, cmd.motor_velocity_rpm);
    }
    // Handle Power Commands  
    else if (frame.id == ID_MOTOR_POWER_COMMAND) {
        MotorPowerCommand cmd{frame};
        state.power_command_bus_current_limit_percent = cmd.bus_current;
        state.last_power_command_time = pdTICKS_TO_MS(xTaskGetTickCount());
        ESP_LOGI(TAG, "Power Command: %.1f%% bus current limit applied", cmd.bus_current * 100.0f);
    }
    // Handle Reset Commands
    else if (frame.id == ID_RESET_COMMAND) {
        ESP_LOGI(TAG, "Reset Command received - resetting simulation state");
        // Not sure what to do here...
    }
}

void MotorControllerSimulator::sendPeriodicMessages(uint32_t current_time_ms, CanDriver& can_driver) {
    // Send Identification Information every 1000ms
    if (current_time_ms - state.last_identification_time >= 1000) {
        IdentificationInformation id_info{};
        id_info.prohelion_ID = state.prohelion_ID;
        id_info.serial_number = state.serial_number;

        can_driver.transmit(pack(id_info));
        state.last_identification_time = current_time_ms;
    }
    
    // Send Status Information every 200ms
    if (current_time_ms - state.last_status_info_time >= 200) {
        StatusInformation status{};
        status.limit_flags = state.limit_flags;  
        status.error_flags = state.error_flags;
        status.active_motor = state.active_motor;
        status.transmit_error_count = state.transmit_error_count;
        status.receive_error_count = state.receive_error_count;
        
        can_driver.transmit(pack(status));
        state.last_status_info_time = current_time_ms;
    }
    
    // Send Bus Measurement every 200ms
    if (current_time_ms - state.last_bus_measurement_time >= 200) {
        BusMeasurement measurement{};
        measurement.bus_voltage = state.bus_voltage_v;
        measurement.bus_current = state.bus_current_a;
        
        can_driver.transmit(pack(measurement));
        state.last_bus_measurement_time = current_time_ms;
    }
    
    // Send Velocity Measurement every 200ms
    if (current_time_ms - state.last_velocity_measurement_time >= 200) {
        VelocityMeasurement measurement{};
        measurement.motor_velocity_rpm = state.motor_velocity_rpm;
        measurement.vehicle_velocity = state.vehicle_velocity_m_s;

        can_driver.transmit(pack(measurement));
        state.last_velocity_measurement_time = current_time_ms;
    }

    // Send Phase Current Measurement every 200ms
    if (current_time_ms - state.last_phase_current_measurement_time >= 200) {
        PhaseCurrentMeasurement measurement{};
        measurement.phase_b_current = state.motor_phase_b_current;
        measurement.phase_c_current = state.motor_phase_c_current;

        can_driver.transmit(pack(measurement));
        state.last_phase_current_measurement_time = current_time_ms;
    }

    // Send Motor Voltage Vector Measurement every 200ms
    if (current_time_ms - state.last_motor_voltage_vector_measurement_time >= 200) {
        MotorVoltageVectorMeasurement measurement{};
        measurement.v_q = state.motor_v_q;
        measurement.v_d = state.motor_v_d;

        can_driver.transmit(pack(measurement));
        state.last_motor_voltage_vector_measurement_time = current_time_ms;
    }

    // Send Motor Current Vector Measurement every 200ms
    if (current_time_ms - state.last_motor_current_vector_measurement_time >= 200) {
        MotorCurrentVectorMeasurement measurement{};
        measurement.i_q = state.motor_i_q;
        measurement.i_d = state.motor_i_d;

        can_driver.transmit(pack(measurement));
        state.last_motor_current_vector_measurement_time = current_time_ms;
    }

    // Send Motor BackEMF Measurement every 200ms
    if (current_time_ms - state.last_motor_backemf_time >= 200) {
        MotorBackEMFMeasurement measurement{};
        measurement.BEMf_q = state.bemf_q;
        measurement.BEMf_d = state.bemf_d;

        can_driver.transmit(pack(measurement));
        state.last_motor_backemf_time = current_time_ms;
    }

    // Send 15V Voltage Rail Measurement every 1000ms    
    if (current_time_ms - state.last_15v_rail_time >= 1000) {
        VoltageRailMeasurement15V measurement{};
        measurement.supply_of_15V = state.supply_of_15v;

        can_driver.transmit(pack(measurement));
        state.last_15v_rail_time = current_time_ms;
    }

    // Send 3V3 and 1V9 Voltage Rail Measurement every 1000ms
    if (current_time_ms - state.last_3v3_1v9_rail_time >= 1000) {
        VoltageRailMeasurement3V3And1V9 measurement{};
        measurement.supply_of_1V9 = state.supply_of_1v9;
        measurement.supply_of_3V3 = state.supply_of_3v3;
        

        can_driver.transmit(pack(measurement));
        state.last_3v3_1v9_rail_time = current_time_ms;
    }

    // Send Heat Sink and Motor Temperature Measurement every 1000ms
    if (current_time_ms - state.last_heatsink_motor_temp_time >= 1000) {
        HeatsinkAndMotorTemperatureMeasurement measurement{};
        measurement.motor_temp = state.motor_temperature_c;
        measurement.heat_sink_temp = state.heat_sink_temperature_c;

        can_driver.transmit(pack(measurement));
        state.last_heatsink_motor_temp_time = current_time_ms;
    }

    // Send DSP Board Temperature Measurement every 1000ms
    if (current_time_ms - state.last_dsp_board_temp_time >= 1000) {
        DSPBoardTemperatureMeasurement measurement{};
        measurement.DSP_board_temp = state.dsp_board_temperature_c;

        can_driver.transmit(pack(measurement));
        state.last_dsp_board_temp_time = current_time_ms;
    }

    // Send Odometer and Bus Amp-Hours Measurement every 1000ms
    if (current_time_ms - state.last_odometer_amp_hours_time >= 1000) {
        OdometerAndBusAmpHoursMeasurement measurement{};
        measurement.odometer = state.odometer_m;
        measurement.DC_bus_amp_hours = state.dc_bus_amp_hours;

        can_driver.transmit(pack(measurement));
        state.last_odometer_amp_hours_time = current_time_ms;
    }
}

void MotorControllerSimulator::updateSimulation(uint32_t current_time_ms) {
    // Calculate delta time for physics calculations
    float delta_time_s = 0.01f;  // Default to 10ms if first run
    if (state.last_simulation_update_time > 0) {
        delta_time_s = (current_time_ms - state.last_simulation_update_time) / 1000.0f;
    }
    state.last_simulation_update_time = current_time_ms;
    
    // ---- MOTOR SAFETY: Check for command timeouts (WaveSculptor requirement) ----
    
    // Drive command timeout (250ms)
    bool drive_command_timeout = (current_time_ms - state.last_drive_command_time) > 250;
    if (drive_command_timeout && state.last_drive_command_time > 0) {
        state.target_motor_velocity_rpm = 0.0f;
        state.target_motor_current_percent = 0.0f;
        ESP_LOGW(TAG, "Drive command timeout - motor stopping");
    }
    
    // The following one I made up:
    // Power command timeout (1000ms) - revert to 100% power limit if no recent power command
    bool power_command_timeout = (current_time_ms - state.last_power_command_time) > 1000;
    if (power_command_timeout && state.last_power_command_time > 0) {
        state.power_command_bus_current_limit_percent = 1.0f; // Remove power limiting (1.0 = 100%)
        ESP_LOGW(TAG, "Power command timeout - removing power limits");
    }
    
    // ---- MOTOR VELOCITY DYNAMICS: Simulate realistic acceleration/deceleration ----
    
    // Motor physics limits, simulated maximum RPM based on motor design
    constexpr float MAX_MOTOR_RPM{ 4000.0f }; // For now as I don't really know what else to put
    
    // Limit target RPM to realistic maximum
    float limited_target_rpm{ std::min(fabsf(state.target_motor_velocity_rpm), MAX_MOTOR_RPM) };
    if (state.target_motor_velocity_rpm < 0) {
        limited_target_rpm = -limited_target_rpm;  // Preserve direction for reverse
    }
    
    // Calculate error with limited target
    float velocity_error{ limited_target_rpm - state.motor_velocity_rpm };
    
    // Reduce acceleration at higher speeds (motors lose torque at high RPM)
    float max_acceleration{ 1000.0f };  // RPM per second (realistic for an electric motor)
    float speed_factor{ 1.0f - (fabsf(state.motor_velocity_rpm) / MAX_MOTOR_RPM) };
    speed_factor = std::max(0.1f, speed_factor);  // Minimum 10% acceleration at max speed
    float adjusted_acceleration{ max_acceleration * speed_factor };
    
    if (fabsf(velocity_error) > 1.0f) {  // Only update if significant difference
        float velocity_change = adjusted_acceleration * delta_time_s;
        if (velocity_error > 0) {
            state.motor_velocity_rpm += std::min(velocity_change, velocity_error);
        } else {
            state.motor_velocity_rpm -= std::min(velocity_change, fabsf(velocity_error));
        }
    }
    
    // ---- VEHICLE VELOCITY: Calculate from motor RPM (wheel radius =~ 0.262m) ----
    float wheel_radius_m = 0.262f;
    state.vehicle_velocity_m_s = (state.motor_velocity_rpm * 2.0f * 3.14159f * wheel_radius_m) / 60.0f;
    
    // ---- MOTOR CURRENT AND TORQUE: Simulate current based on load and velocity ----
    float base_current = 2.5f;  // No-load current in amps
    float load_current = fabsf(state.motor_velocity_rpm) * 0.005f;  // Load increases with speed
    
    // Calculate requested current from drive command
    float requested_current = base_current + load_current + (state.target_motor_current_percent * 50.0f);
    
    // Apply power limiting from Power Command (global system power limiter)
    float max_allowed_current = 100.0f;  // Absolute maximum current rating (100A example) I think this is appropriate for a WaveSculptor
    float power_limited_current = max_allowed_current * state.power_command_bus_current_limit_percent; // 1.0 = 100%, 0.5 = 50%
    
    // Final current is the minimum of requested vs power-limited
    state.bus_current_a = std::min(requested_current, power_limited_current);
    
    // Log if power limiting is active
    if (requested_current > power_limited_current) {
        static uint32_t last_power_limit_log = 0;
        if (current_time_ms - last_power_limit_log > 2000) {  // Log every 2 seconds
            ESP_LOGW(TAG, "Power limiting active: requested %.1fA, limited to %.1fA (%.1f%% power limit)", 
                     requested_current, power_limited_current, state.power_command_bus_current_limit_percent * 100.0f);
            last_power_limit_log = current_time_ms;
        }
    }
    
    // ---- PHASE CURRENTS: Update based on motor load ----
    float phase_current_multiplier = 1.6f;  // Phase current typically higher than DC bus current
    state.motor_phase_b_current = state.bus_current_a * phase_current_multiplier;
    state.motor_phase_c_current = state.motor_phase_b_current;  // Balanced 3-phase
    
    // ---- MOTOR HEATING: Simulate temperature rise based on current ----
    float heat_generation = state.bus_current_a * state.bus_current_a * 0.001f;  // I²R heating
    float cooling_rate = (state.motor_temperature_c - 25.0f) * 0.02f;  // Cooling to ambient (Newton's Law of Cooling)
    state.motor_temperature_c += (heat_generation - cooling_rate) * delta_time_s; // Skips Q = mcΔT for simplicity
    
    // ---- HEATSINK TEMPERATURE: Follows motor temperature with delay ----
    float temp_diff = state.motor_temperature_c - state.heat_sink_temperature_c;
    state.heat_sink_temperature_c += temp_diff * 0.1f * delta_time_s;  // Thermal lag
    
    // ---- BACK-EMF: Calculate based on motor velocity ----
    state.bemf_q = state.motor_velocity_rpm * 0.023f;  // Back-EMF constant
    state.bemf_d = 0.0f;  // d-axis back-EMF typically zero
    
    // ---- MOTOR VOLTAGE VECTORS: Simulate based on current and back-EMF ----
    state.motor_v_q = state.bemf_q + (state.motor_i_q * 0.1f);  // Voltage = back-EMF + I*R
    state.motor_v_d = state.motor_i_d * 0.1f;
    
    // ---- MOTOR CURRENT VECTORS: Update based on torque demand ----
    state.motor_i_q = state.target_motor_current_percent * 50.0f;  // Torque-producing current
    state.motor_i_d = -0.5f;  // Field-weakening current (usually small and negative)
    
    // ---- VOLTAGE RAIL MONITORING: Simulate slight variations ----
    state.supply_of_15v = 14.9f + (sinf(current_time_ms * 0.001f) * 0.1f);  // Small ripple
    state.supply_of_3v3 = 3.31f + (sinf(current_time_ms * 0.002f) * 0.02f);
    state.supply_of_1v9 = 1.91f + (sinf(current_time_ms * 0.003f) * 0.01f);
    
    // ---- DSP BOARD TEMPERATURE: Slight increase with processing load ----
    state.dsp_board_temperature_c = 35.5f + (state.bus_current_a * 0.1f);
    
    // ---- ODOMETER: Accumulate distance traveled ----
    state.odometer_m += state.vehicle_velocity_m_s * delta_time_s;
    
    // ---- AMP-HOURS: Accumulate energy consumption ----
    state.dc_bus_amp_hours += (state.bus_current_a * delta_time_s) / 3600.0f;
    
    // ---- ERROR FLAG MANAGEMENT: Set flags based on operating conditions ----
    state.error_flags = 0;  // Clear all flags first
    
    if (state.motor_temperature_c > 85.0f) {
        state.error_flags |= 0x0001;  // Hardware over current (temperature protection)
    }
    if (state.bus_current_a > 50.0f) {
        state.error_flags |= 0x0002;  // Software over current
    }
    if (state.bus_voltage_v > 110.0f) { // Value can change here depending on the bus_voltage 
        state.error_flags |= 0x0004;  // DC Bus over voltage
    }
    if (state.supply_of_15v < 13.0f) {
        state.error_flags |= 0x0040;  // 15V Rail under voltage lock out
    }
    if (fabsf(state.motor_velocity_rpm) > 4000.0f) {
        state.error_flags |= 0x0100;  // Motor Over Speed
    }
    
    // ---- LIMIT FLAG MANAGEMENT: Indicate what's limiting performance ----
    // Note: Don't clear all flags here as power limiting flag was set earlier
    
    // Clear and set temperature limiting
    state.limit_flags &= ~0x0040;  // Clear temperature limit flag
    if (state.motor_temperature_c > 70.0f) {
        state.limit_flags |= 0x0040;  // IPM Temperature limiting (Bit 6)
    }
    
    // Clear and set high current limiting (different from power command limiting)
    state.limit_flags &= ~0x0008;  // Clear high current limit flag first
    if (requested_current > power_limited_current) {
        state.limit_flags |= 0x0008;  // Bus current limiting due to power command
    } else if (state.bus_current_a > 80.0f) {
        state.limit_flags |= 0x0008;  // Bus current limiting due to high current
    }
    
    // Clear and set velocity limiting
    state.limit_flags &= ~0x0004;  // Clear velocity limit flag
    if (fabsf(state.motor_velocity_rpm) > 3800.0f) {
        state.limit_flags |= 0x0004;  // Velocity limiting (Bit 2)
    }
    
    // Clear and set motor current limiting
    state.limit_flags &= ~0x0002;  // Clear motor current limit flag
    if (state.motor_i_q > 30.0f) {
        state.limit_flags |= 0x0002;  // Motor Current limiting (Bit 1)
    }
    
    // ---- BUS VOLTAGE: Simulate voltage drop under load ----
    float base_voltage = 100.0f; // Could change depending on our real one
    float voltage_drop = state.bus_current_a * 0.2f;  // Internal resistance
    state.bus_voltage_v = base_voltage - voltage_drop;
}

extern "C" void app_main(void)
{
    // Initialize CAN driver
    CanDriver can_driver;
    can_driver.initialize();
    
    // Create motor controller simulator
    MotorControllerSimulator motor_sim;
    
    ESP_LOGI(TAG, "WaveSculptor Motor Controller Simulator started");

    while (true) {
        uint32_t current_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        
        // Process incoming commands
        CanFrame received_frame;
        if (can_driver.receive(received_frame, 1) == Result::R_SUCCESS) {
            motor_sim.processIncomingCommand(received_frame);
        }
        
        // Update simulation physics
        motor_sim.updateSimulation(current_time_ms);
        
        // Send periodic messages
        motor_sim.sendPeriodicMessages(current_time_ms, can_driver);
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Prevent watchdog timeout
    }
}