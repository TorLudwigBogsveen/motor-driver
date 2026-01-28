#ifndef CAN_DRIVER_HPP
#define CAN_DRIVER_HPP

#include "can_frame.hpp"

/// @brief CAN driver abstraction
class CanDriver {
public:
    /// @brief Initialize the CAN peripheral
    /// @return R_SUCCESS on success, R_ERROR on failure
    Result initialize();

    /// @brief Transmit a CAN frame
    /// @param frame The frame to transmit
    /// @return Result code indicating success or failure
    Result transmit(const CanFrame& frame);

    /// @brief Receive a CAN frame
    /// @param out_frame Output parameter for received frame
    /// @param timeout_ms Timeout in milliseconds (0 = no wait)
    /// @return R_SUCCESS with valid frame, R_NO_MESSAGE on timeout, R_ERROR on failure
    Result receive(CanFrame& out_frame, uint32_t timeout_ms = 0);
};

#endif // CAN_DRIVER_HPP
