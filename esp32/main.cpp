// # Copyright (c) 2026 MDU Solar Team
#include "motor_driver.hpp"
#include "can/can_driver.hpp"
#include "can/can_protocol.hpp"
#include "controller.hpp"
#include "clock.hpp"
#include "log.hpp"

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

extern "C" void app_main(void) {
    can_driver.initialize();
    controller_clock.busyWait(1000);
    controller_clock.start();

    while (true)
    {
        loop();
    }
}