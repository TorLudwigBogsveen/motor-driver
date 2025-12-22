cmake_minimum_required(VERSION 3.5)
project(MotorDriver LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable (or library if you prefer)
add_executable(${PROJECT_NAME}
    ../core/error.cpp
    ../core/input.cpp
    ../core/controller.cpp
    ../core/wavesculptor_simulator.cpp
    ../windows/main.cpp
    ../windows/can_driver.cpp
    ../windows/clock.cpp
    ../windows/log.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ../include
)

# Platform‑specific definitions (optional)
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        PLATFORM_WINDOWS=1
)