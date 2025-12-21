#include "can_driver.hpp"
#include <windows.h>
#include <iostream>

HANDLE sendPipe;
HANDLE recvPipe;

Result CanDriver::initialize() {
    const int bufferSize = 1 << 16;
    const char* sendPipeName = R"(\\.\pipe\CANPipe_CtoP)";
    const char* recvPipeName = R"(\\.\pipe\CANPipe_PtoC)";

    sendPipe = CreateNamedPipeA(
        sendPipeName,
        PIPE_ACCESS_OUTBOUND,
        PIPE_TYPE_BYTE | PIPE_WAIT,
        PIPE_UNLIMITED_INSTANCES, bufferSize, 0, 0, nullptr
    );
    if(sendPipe == INVALID_HANDLE_VALUE) {
        std::cout << "Failed to create send named pipe." << std::endl;
        return Result::R_ERROR;
    }

    recvPipe = CreateNamedPipeA(
        recvPipeName,
        PIPE_ACCESS_INBOUND,
        PIPE_TYPE_BYTE | PIPE_WAIT,
        PIPE_UNLIMITED_INSTANCES, 0, bufferSize, 0, nullptr
    );

    if(recvPipe == INVALID_HANDLE_VALUE) {
        std::cout << "Failed to create receive named pipe." << std::endl;
        return Result::R_ERROR;
    }

    std::cout << "Waiting for Python client to connect..." << std::endl;
    if(!ConnectNamedPipe(sendPipe, nullptr)) {
        std::cout << "Failed to connect to send named pipe." << std::endl;
        return Result::R_ERROR;
    }

    if(!ConnectNamedPipe(recvPipe, nullptr)) {
        std::cout << "Failed to connect to receive named pipe." << std::endl;
        return Result::R_ERROR;
    }

    std::cout << "Client connected. Sending CAN messages..." << std::endl;
    return Result::R_SUCCESS;
}

Result CanDriver::transmit(const CanFrame& frame) {

    uint8_t dlc = 8;
    DWORD bytesWritten;
    // Write CAN ID
    if(!WriteFile(sendPipe, &frame.id, sizeof(frame.id), &bytesWritten, nullptr))
        return Result::R_ERROR;
    // Write DLC
    if(!WriteFile(sendPipe, &dlc, sizeof(dlc), &bytesWritten, nullptr))
        return Result::R_ERROR;
    // Write Data
    if(!WriteFile(sendPipe, frame.data, sizeof(frame.data), &bytesWritten, nullptr))
        return Result::R_ERROR;

    std::cout << "Sent CAN Frame - ID: " << frame.id << " DLC: " << static_cast<int>(dlc) << " Data: ";
    for(int i = 0; i < dlc; i++) {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
    return Result::R_SUCCESS;
}

Result CanDriver::receive(CanFrame& out_frame, uint32_t timeout_ms) {
    uint8_t receiveBuffer[13]; // 4 bytes ID + 1 byte DLC + 8 bytes data
    DWORD bytesRead;
    BOOL result = ReadFile(recvPipe, receiveBuffer, sizeof(receiveBuffer), &bytesRead, nullptr);
    if(!result || bytesRead == 0) {
        return Result::R_NO_MESSAGE;
    }
    if(bytesRead < 5) { // At least ID + DLC
        return Result::R_ERROR;
    }
    // Parse CAN ID
    out_frame.id = *reinterpret_cast<uint32_t*>(receiveBuffer);
    // Parse DLC
    uint8_t dlc = receiveBuffer[4];
    if(dlc > 8) {
        return Result::R_ERROR;
    }
    // Parse Data
    for(int i = 0; i < dlc; i++) {
        out_frame.data[i] = receiveBuffer[5 + i];
    }
    out_frame.dlc = dlc;
    return Result::R_SUCCESS;
}