cmake_minimum_required(VERSION 3.5)

#set(ESP_PLATFORM TRUE)

# ESP-IDF build
if(ESP_PLATFORM)
    set(EXTRA_COMPONENT_DIRS
        "${CMAKE_SOURCE_DIR}/core"
        "${CMAKE_SOURCE_DIR}/esp32"
    )

    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(MotorDriver)

# Desktop / Windows build
else()
    project(MotorDriver)
    add_subdirectory(windows)
endif()
