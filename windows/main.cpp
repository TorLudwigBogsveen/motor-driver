// # Copyright (c) 2025 MDU Solar Team
#include <iostream>
#include "motor_driver.hpp"
#include "can_driver.hpp"
#include "protocol.hpp"
#include "controller.hpp"
#include "clock.hpp"

CanDriver canDriver;
Controller controller;

Clock controllerClock;
Clock driveCommandClock;

void readCanMessages(Controller &controller)
{
    CanFrame m;
    if (canDriver.receive(m) == Result::R_SUCCESS)
    {
        switch (m.id)
        {
        case ID_STATUS_INFORMATION:
        {
            StatusInformation status = StatusInformation(m);
            controller.setError(static_cast<MotorFlags>(status.error_flags & 0x1ff)); // mask to remove unwanted reserved bits
            controller.setLimit(status.limit_flags & 0x7f);  // mask to remove unwanted reserved bits
            break;
        }
        case ID_BUS_MEASUREMENT:
        {
            BusMeasurement bm = BusMeasurement(m);
            controller.setBusCurrent(bm.bus_current);
            break;
        }
        case ID_VELOCITY_MEASUREMENT:
        {
            VelocityMeasurement vm = VelocityMeasurement(m);
            controller.setVelocity(vm.vehicle_velocity);
            controller.setMotorVelocity(vm.motor_velocity_rpm);
            break;
        }
        case ID_PHASE_CURRENT_MEASUREMENT:
        {
            PhaseCurrentMeasurement pcm = PhaseCurrentMeasurement(m);
            break;
        }
        case ID_MOTOR_VOLTAGE_VECTOR_MEASUREMENT:
        {
            MotorVoltageVectorMeasurement vvm = MotorVoltageVectorMeasurement(m);
            break;
        }
        case ID_MOTOR_CURRENT_VECTOR_MEASUREMENT:
        {
            MotorCurrentVectorMeasurement cvm = MotorCurrentVectorMeasurement(m);

            break;
        }
        case ID_HEATSINK_AND_MOTOR_TEMPERATURE_MEASUREMENT:
        {
            HeatsinkAndMotorTemperatureMeasurement temp = HeatsinkAndMotorTemperatureMeasurement(m);
            controller.setHeatSinkTemp(temp.heat_sink_temp);
            controller.setMotorTemp(temp.motor_temp);
            break;
        }
        case ID_DSP_BOARD_TEMPERATURE_MEASUREMENT:
        {
            DSPBoardTemperatureMeasurement temp = DSPBoardTemperatureMeasurement(m);
            controller.setDspBoardTemp(temp.DSP_board_temp);
            break;
        }
        case ID_ODOMETER_AND_BUS_AMP_HOURS_MEASUREMENT:
        {
            OdometerAndBusAmpHoursMeasurement om = OdometerAndBusAmpHoursMeasurement(m);
            controller.setOdometer(om.odometer);
            break;
        }
        case ID_SLIP_SPEED_MEASUREMENT:
        {
            SlipSpeedMeasurement ssm = SlipSpeedMeasurement(m);
            break;
        }
        default:
            // TODO FIX
            break;
        }
    }
}

void setup()
{
    std::cout << "Initializing CAN Driver..." << std::endl;
    canDriver.initialize();
    controllerClock.busyWait(1000);
    controllerClock.start();
    driveCommandClock.start();
}

//********************************Main Loop*********************************//

void loop()
{
    readCanMessages(controller);

    auto &buttons = controller.getButtonsMut();
    buttons.update();
    auto &sliders = controller.getSlidersMut();
    // sliders.set(ACCELERATION_POTENTIOMETER, ioState.getPotentiometerValue(ACCELERATION_POTENTIOMETER));
    // sliders.set(REGENERATION_POTENTIOMETER, ioState.getPotentiometerValue(REGENERATION_POTENTIOMETER));
    int oldVal = sliders.get(0);
    // int val = min(max(analogRead(34) - 1000, 0) / 3, MAX_POT_VALUE);
    // sliders.set(0, val);
    controller.update(controllerClock.elapsed_millis(), controllerClock.elapsed_micros());
    if(driveCommandClock.elapsed_millis() >= 500) {
        driveCommandClock.start();

        sliders.set(0, oldVal + 100);

        SpeedCommand command = controller.motorCommand();
        MotorDriveCommand cmd(command.current, command.velocity);
        canDriver.transmit(pack(cmd));
        std::cout << "Motor Driver Running..." << std::endl;
        std::cout << "Target Current: " << command.current << " Target Velocity: " << command.velocity << std::endl;
        std::cout << "Actual Velocity: " << controller.getMotorVelocity() << " Odometer: " << controller.getOdometer() << std::endl;
        std::cout << "Heat Sink Temp: " << controller.getHeatSinkTemp() << " Motor Temp: " << controller.getMotorTemp() << " DSP Temp: " << controller.getDspBoardTemp() << std::endl;
        std::cout << "Error Code: " << static_cast<int>(controller.getError()) << " Limit Code: " << controller.getLimit() << std::endl;
        std::cout << "Drive Mode: " << int(controller.getMode()) << " Direction: " << int(controller.getDirection()) << std::endl;
        std::cout << "Controller State: " << int(controller.getState()) << std::endl;
        std::cout << "-----------------------------" << std::endl;

    }
}

int main()
{
    setup();
    while (true)
    {
        loop();
    }
    return 0;
}