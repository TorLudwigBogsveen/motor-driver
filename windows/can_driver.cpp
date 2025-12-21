#include "can_driver.hpp"
#include <windows.h>
#include <iostream>

HANDLE hPipe;

Result CanDriver::initialize() {
    const int bufferSize = 2 << 15;
    const char* pipeName = R"(\\.\pipe\CANPipe_CtoP)";

    hPipe = CreateNamedPipeA(
        pipeName,
        PIPE_ACCESS_OUTBOUND,
        PIPE_TYPE_BYTE | PIPE_WAIT,
        PIPE_UNLIMITED_INSTANCES, bufferSize, 0, 0, nullptr
    );

    if(hPipe == INVALID_HANDLE_VALUE)  {
        std::cout << "Failed to create named pipe." << std::endl;
        return Result::R_ERROR;
    }

    std::cout << "Waiting for Python client to connect..." << std::endl;
    if(!ConnectNamedPipe(hPipe, nullptr))
        return Result::R_ERROR;

    std::cout << "Client connected. Sending CAN messages..." << std::endl;
    return Result::R_SUCCESS;
}

Result CanDriver::transmit(const CanFrame& frame) {

    uint8_t dlc = 8;
    DWORD bytesWritten;
    // Write CAN ID
    if(!WriteFile(hPipe, &frame.id, sizeof(frame.id), &bytesWritten, nullptr))
        return Result::R_ERROR;
    // Write DLC
    if(!WriteFile(hPipe, &dlc, sizeof(dlc), &bytesWritten, nullptr))
        return Result::R_ERROR;
    // Write Data
    if(!WriteFile(hPipe, frame.data, sizeof(frame.data), &bytesWritten, nullptr))
        return Result::R_ERROR;

    std::cout << "Sent CAN Frame - ID: " << frame.id << " DLC: " << static_cast<int>(dlc) << " Data: ";
    for(int i = 0; i < dlc; i++) {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
    return Result::R_SUCCESS;
}

Result CanDriver::receive(CanFrame& out_frame, uint32_t timeout_ms) {
    return Result::R_SUCCESS;
}