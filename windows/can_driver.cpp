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
        PIPE_TYPE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
        PIPE_UNLIMITED_INSTANCES,
        bufferSize, bufferSize, 0, nullptr
    );
    
    if (sendPipe == INVALID_HANDLE_VALUE) {
        std::cout << "Failed to create send named pipe. Error: "
                << GetLastError() << std::endl;
        return Result::R_ERROR;
    }

    recvPipe = CreateNamedPipeA(
        recvPipeName,
        PIPE_ACCESS_INBOUND,
        PIPE_TYPE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
        PIPE_UNLIMITED_INSTANCES,
        bufferSize, bufferSize, 0, nullptr
    );

    if (recvPipe == INVALID_HANDLE_VALUE) {
        CloseHandle(sendPipe);
        std::cout << "Failed to create receive named pipe. Error: "
                << GetLastError() << std::endl;
        return Result::R_ERROR;
    }

    std::cout << "Waiting for Python client to connect..." << std::endl;

    if (!ConnectNamedPipe(sendPipe, nullptr)) {
        if (GetLastError() != ERROR_PIPE_CONNECTED) {
            std::cout << "Failed to connect send pipe. Error: "
                    << GetLastError() << std::endl;
            return Result::R_ERROR;
        }
    }

    std::cout << "Send pipe connected" << std::endl;

    if (!ConnectNamedPipe(recvPipe, nullptr)) {
        if (GetLastError() != ERROR_PIPE_CONNECTED) {
            std::cout << "Failed to connect receive pipe. Error: "
                    << GetLastError() << std::endl;
            return Result::R_ERROR;
        }
    }

    std::cout << "Receive pipe connected" << std::endl;
    return Result::R_SUCCESS;
}

Result CanDriver::transmit(const CanFrame& frame) {

    uint8_t dlc = 8;
    DWORD bytesWritten;
    uint8_t buffer[13];
    memcpy(buffer, &frame.id, 4);
    buffer[4] = dlc;
    memcpy(buffer + 5, frame.data, 8);

    if (!WriteFile(
            sendPipe,
            buffer,
            13,
            &bytesWritten,
            nullptr))
    {
        std::cout << "WriteFile failed. Error: "
                << GetLastError() << std::endl;
        return Result::R_ERROR;
    }

    //std::cout << "Sent CAN Frame - ID: " << frame.id << " DLC: " << static_cast<int>(dlc) << " Data: ";
    //for(int i = 0; i < dlc; i++) {
        //std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    //}
    //std::cout << std::dec << std::endl;
    return Result::R_SUCCESS;
}

Result CanDriver::receive(CanFrame& out_frame, uint32_t /*timeout_ms*/) {
    constexpr DWORD FRAME_SIZE = 4 + 1 + 8; // ID + DLC + data
    uint8_t receiveBuffer[FRAME_SIZE];

    DWORD bytesAvailable = 0;
    if (!PeekNamedPipe(
            recvPipe,
            nullptr,
            0,
            nullptr,
            &bytesAvailable,
            nullptr))
    {
        std::cout << "PeekNamedPipe failed. Error: "
                  << GetLastError() << std::endl;
        return Result::R_ERROR;
    }

    // No complete frame available → non-blocking return
    if (bytesAvailable < FRAME_SIZE) {
        return Result::R_NO_MESSAGE;
    }

    DWORD bytesRead = 0;
    if (!ReadFile(
            recvPipe,
            receiveBuffer,
            FRAME_SIZE,
            &bytesRead,
            nullptr))
    {
        std::cout << "ReadFile failed. Error: "
                  << GetLastError() << std::endl;
        return Result::R_ERROR;
    }

    if (bytesRead != FRAME_SIZE) {
        return Result::R_ERROR;
    }

    // Parse CAN ID
    out_frame.id =
        static_cast<uint32_t>(receiveBuffer[0]) |
        (static_cast<uint32_t>(receiveBuffer[1]) << 8) |
        (static_cast<uint32_t>(receiveBuffer[2]) << 16) |
        (static_cast<uint32_t>(receiveBuffer[3]) << 24);

    // Parse DLC
    uint8_t dlc = receiveBuffer[4];
    if (dlc > 8) {
        return Result::R_ERROR;
    }

    out_frame.dlc = dlc;

    // Parse data
    for (uint8_t i = 0; i < dlc; ++i) {
        out_frame.data[i] = receiveBuffer[5 + i];
    }

    return Result::R_SUCCESS;
}