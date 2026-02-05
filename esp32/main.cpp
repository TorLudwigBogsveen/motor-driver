// # Copyright (c) 2026 MDU Solar Team
#include "motor_driver.hpp"
#include "can/can_driver.hpp"
#include "can/can_protocol.hpp"
#include "controller.hpp"
#include "clock.hpp"
#include "log.hpp"
#include "ResponsiveAnalogRead.h"

CanDriver can_driver;
Controller controller;

Clock controller_clock;

//********************************Main Loop*********************************//

void loop()
{
    CanFrame m;
    while (can_driver.receive(m) == Result::R_SUCCESS)
    {
        controller.processIncomingCommand(m);
    }

    controller.update(static_cast<uint32_t>(controller_clock.elapsed_millis()), static_cast<uint32_t>(controller_clock.elapsed_micros()));
    controller.setDirection(MotorDirection::Forward);

    auto con_can = [](const CanFrame &frame, void *t_state)
    {
        CanDriver *driver = static_cast<CanDriver *>(t_state);
        driver->transmit(frame);
    };

    // Send periodic messages
    controller.sendPeriodicMessages(static_cast<uint32_t>(controller_clock.elapsed_millis()), &can_driver, con_can);
    controller.setMode(DriveMode::Velocity);

    controller_clock.busyWait(100); // Prevent watchdog timeout
}

#include "driver/uart.h"
#include "driver/adc.h"
#include "esp32/rom/ets_sys.h"
#include "esp_timer.h"

void initializeSerial() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

uint32_t readAnalogInput(adc1_channel_t channel) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);
    return adc1_get_raw(channel);
}

void sendSerialMessage(const char *message) {
    uart_write_bytes(UART_NUM_0, message, strlen(message));
}

ResponsiveAnalogRead a;

void IRAM_ATTR adc_timer_callback(void* arg) {
    int raw = adc1_get_raw(ADC1_CHANNEL_0);
    a.update(raw);
}

esp_timer_handle_t adc_timer;

void start_adc_timer() {
    a.setAnalogResolution(4096);
    a.setSnapMultiplier(0.1f);
    a.enableSleep();
    a.setActivityThreshold(16.0f);
    
    esp_timer_create_args_t timer_args = {
        .callback = &adc_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "adc_timer"
    };

    esp_timer_create(&timer_args, &adc_timer);

    // 1000 µs = 1 kHz
    esp_timer_start_periodic(adc_timer, 1000);
}


extern "C" void app_main(void) {
    /*can_driver.initialize();
    controller_clock.busyWait(1000);
    controller_clock.start();

    while (true)
    {
        loop();
    }*/
    initializeSerial();
    start_adc_timer();
    while (true) {
        float v = a.getValueQuantized(8, 6); // Quantize to 8 steps with hysteresis of 6

        // Convert to percentage
        float percentage = (v / 4095.0f) * 100.0f;

        if (percentage > 100.0f) percentage = 100.0f;
        if (percentage < 0.0f) percentage = 0.0f;

        // Output
        char buffer[32];
        sprintf(buffer, "%f\n", percentage * 1.6f);
        sendSerialMessage(buffer);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}