#include "can_driver.hpp"
#include "protocol.hpp"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

static const char* TAG = "MOTOR_SIM";

extern "C" void app_main(void)
{
    // Initialize CAN driver
    CanDriver can_driver;
    can_driver.initialize();
    
    // Motor controller state
    int16_t target_torque{ 0 };
    uint32_t last_measurement_time{ 0 };
    
    ESP_LOGI(TAG, "Motor Controller started");

    while (true) {
        uint32_t current_time{ pdTICKS_TO_MS(xTaskGetTickCount()) };
        
        // Receive drive commands
        CanFrame cmd_frame;
        if (can_driver.receive(cmd_frame, 1) == Result::R_SUCCESS) {
            if (cmd_frame.id == ID_DRIVE_COMMAND) {
                DriveCommand command{ cmd_frame };
                target_torque = command.torque_request;
                if (command.direction == 1) {
                    target_torque = -target_torque;
                }
                ESP_LOGI(TAG, "New torque: %d Nm", target_torque);
            }
        }

        // Send measurements every 50ms
        if (current_time - last_measurement_time >= 50) {
            BusMeasurement measurement{};
            measurement.current_mA = 10000 + (target_torque * 10); // 10000mA is the no-load current
            measurement.voltage_mV = 48000 - (measurement.current_mA / 100); // 47900mV is the nominal voltage
            
            can_driver.transmit(pack(measurement));
            last_measurement_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Give CPU time to other tasks so that WDT doesn't trigger
    }
}