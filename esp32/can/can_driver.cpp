#include "can/can_driver.hpp"
#include "driver/twai.h"

twai_message_t toTwaiMessage(const CanFrame& frame);
CanFrame fromTwaiMessage(const twai_message_t& msg);

Result CanDriver::initialize() {
    // Configure TWAI timing (500 kbps for typical automotive CAN)
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();

    // Configure TWAI filter to accept all messages
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Configure TWAI general settings (adjust GPIO pins as needed)
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(
        GPIO_NUM_21,  // TX pin
        GPIO_NUM_22,  // RX pin
        TWAI_MODE_NORMAL
    );

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&general_config, &timing_config, &filter_config);
    if (err != ESP_OK) {
        return Result::R_ERROR;
    }

    // Start TWAI driver
    err = twai_start();
    if (err != ESP_OK) {
        twai_driver_uninstall(); // Cleanup on failure
        return Result::R_ERROR;
    }

    return Result::R_SUCCESS;
}

Result CanDriver::transmit(const CanFrame& frame) {
    twai_message_t msg = toTwaiMessage(frame);

    // Transmit with 10ms timeout
    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));

    switch (err) {
    case ESP_OK:
        return Result::R_SUCCESS;
    case ESP_ERR_TIMEOUT:
        return Result::R_TIMEOUT;
    default:
        return Result::R_ERROR;
    }
}

Result CanDriver::receive(CanFrame& out_frame, uint32_t timeout_ms) {
    twai_message_t msg;

    // Convert timeout to FreeRTOS ticks
    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);

    esp_err_t err = twai_receive(&msg, ticks);

    switch (err) {
    case ESP_OK:
        out_frame = fromTwaiMessage(msg);
        return Result::R_SUCCESS;
    case ESP_ERR_TIMEOUT:
        return Result::R_NO_MESSAGE;
    default:
        return Result::R_ERROR;
    }
}

twai_message_t toTwaiMessage(const CanFrame& frame) {
    twai_message_t msg{};
    msg.identifier = frame.id;
    msg.data_length_code = frame.dlc;
    msg.extd = 0;  // Standard 11-bit identifier
    msg.rtr = 0;   // Data frame
    msg.ss = 0;    // Not single shot
    msg.self = 0;  // Not self-reception
    msg.dlc_non_comp = 0;

    for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
        msg.data[i] = frame.data[i];
    }

    return msg;
}

CanFrame fromTwaiMessage(const twai_message_t& msg) {
    CanFrame frame{};
    frame.id = msg.identifier;
    frame.dlc = msg.data_length_code;

    for (uint8_t i = 0; i < msg.data_length_code && i < 8; ++i) {
        frame.data[i] = msg.data[i];
    }

    return frame;
}
