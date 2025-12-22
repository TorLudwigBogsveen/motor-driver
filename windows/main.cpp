// # Copyright (c) 2025 MDU Solar Team
#include <iostream>
#include "motor_driver.hpp"
#include "can_driver.hpp"
#include "protocol.hpp"
#include "controller.hpp"
#include "clock.hpp"
#include "log.hpp"
#include "wavesculptor_simulator.hpp"

CanDriver can_driver;
Controller controller;

// Create motor controller simulator
MotorControllerSimulator motor_sim;

Clock controller_clock;

//********************************Main Loop*********************************//

void loop()
{
    CanFrame m;
    while (can_driver.receive(m) == Result::R_SUCCESS)
    {
        controller.processIncomingCommand(m);
        motor_sim.processIncomingCommand(m);
    }

    // Update simulation physics
    motor_sim.updateSimulation(static_cast<uint32_t>(controller_clock.elapsed_millis()));

    auto sim_can = [](const CanFrame &frame, void *t_state)
    {
        CanDriver *driver = static_cast<CanDriver *>(t_state);
        driver->transmit(frame);
        controller.processIncomingCommand(frame);
    };

    // Send periodic messages
    motor_sim.sendPeriodicMessages(static_cast<uint32_t>(controller_clock.elapsed_millis()), &can_driver, sim_can);

    auto &buttons = controller.getButtonsMut();
    buttons.update();
    auto &sliders = controller.getSlidersMut();
    // sliders.set(ACCELERATION_POTENTIOMETER, ioState.getPotentiometerValue(ACCELERATION_POTENTIOMETER));
    // sliders.set(REGENERATION_POTENTIOMETER, ioState.getPotentiometerValue(REGENERATION_POTENTIOMETER));
    int oldVal = sliders.getFloat(0);
    // int val = min(max(analogRead(34) - 1000, 0) / 3, MAX_POT_VALUE);
    // sliders.set(0, val);
    controller.update(static_cast<uint32_t>(controller_clock.elapsed_millis()), static_cast<uint32_t>(controller_clock.elapsed_micros()));
    controller.setDirection(MotorDirection::Forward);

    auto con_can = [](const CanFrame &frame, void *t_state)
    {
        CanDriver *driver = static_cast<CanDriver *>(t_state);
        driver->transmit(frame);
        motor_sim.processIncomingCommand(frame);
    };

    // Send periodic messages
    controller.sendPeriodicMessages(static_cast<uint32_t>(controller_clock.elapsed_millis()), &can_driver, con_can);
    controller.setMode(DriveMode::Velocity);

    static int pot = 0;
    pot = pot + 1;
    int v = (pot / 10) % 1024;
    float fv = static_cast<float>(v) / 1023.0f;
    sliders.set(0, fv);
    std::cout << "Potentiometer value: " << sliders.getFloat(0) << ": Should be " << fv << std::endl;

    controller_clock.busyWait(10); // Prevent watchdog timeout
}

int main()
{
    std::cout << "Initializing CAN Driver..." << std::endl;
    can_driver.initialize();
    controller_clock.busyWait(1000);
    controller_clock.start();
    log("MOTOR_SIM", "WaveSculptor Motor Controller Simulator started");

    while (true)
    {
        loop();
    }

    return 0;
}