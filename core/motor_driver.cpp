//# Copyright (c) 2025 MDU Solar Team
#include "motor_driver.hpp"
#include "can_driver.hpp"
#include "protocol.hpp"
#include "controller.hpp"

CanDriver can_driver;
Controller controller;
unsigned long lastTime;
unsigned long lastSerialTime;
unsigned long lastDriveCommandTime;

void readCanMessages(Controller& controller) {
  CanFrame m;
  if (can_driver.receive(m) == Result::R_SUCCESS) {
    switch(m.id) {
      case ID_STATUS_INFORMATION: {
        StatusInformation status = StatusInformation(m);
        controller.setError(status.error_flags & 0x1ff); //mask to remove unwanted reserved bits
        controller.setLimit(status.limit_flags & 0x7f); //mask to remove unwanted reserved bits
        break;
      }
      case ID_BUS_MEASUREMENT: {
        BusMeasurement bm = BusMeasurement(m);
        controller.setBusCurrent(bm.bus_current);
        break;                  
      }       
      case ID_VELOCITY_MEASUREMENT: {
        VelocityMeasurement vm = VelocityMeasurement(m);
        controller.setVelocity(vm.vehicle_velocity);
        controller.setMotorVelocity(vm.motor_velocity_rpm);
        break;                    
      }
      case ID_PHASE_CURRENT_MEASUREMENT: {
        PhaseCurrentMeasurement pcm = PhaseCurrentMeasurement(m);
        break;                
      }
      case ID_MOTOR_VOLTAGE_VECTOR_MEASUREMENT: {
        MotorVoltageVectorMeasurement vvm = MotorVoltageVectorMeasurement(m);
        break;               
      }
      case ID_MOTOR_CURRENT_VECTOR_MEASUREMENT: {
        MotorCurrentVectorMeasurement cvm = MotorCurrentVectorMeasurement(m);
        
        break;               
      }
      case ID_HEATSINK_AND_MOTOR_TEMPERATURE_MEASUREMENT: {
        HeatsinkAndMotorTemperatureMeasurement temp = HeatsinkAndMotorTemperatureMeasurement(m);
        controller.setHeatSinkTemp(temp.heat_sink_temp);
        controller.setMotorTemp(temp.motor_temp);
        break;    
      }
      case ID_DSP_BOARD_TEMPERATURE_MEASUREMENT: {
        DSPBoardTemperatureMeasurement temp = DSPBoardTemperatureMeasurement(m);
        controller.setDspBoardTemp(temp.DSP_board_temp);
        break;         
      }     
      case ID_ODOMETER_AND_BUS_AMP_HOURS_MEASUREMENT: {
        OdometerAndBusAmpHoursMeasurement om = OdometerAndBusAmpHoursMeasurement(m);
        controller.setOdometer(om.odometer);
        break;         
      }
      case ID_SLIP_SPEED_MEASUREMENT: {
        SlipSpeedMeasurement ssm = SlipSpeedMeasurement(m);
        break;                   
      }
      default:
        //TODO FIX
        break;
    }
  }
}

void setup() {
  can_driver.initialize();
  //delay(1000);

  //lastTime = millis();
  //lastSerialTime = millis();
  //lastDriveCommandTime = millis();
}

//********************************Main Loop*********************************//

void loop() {
  readCanMessages(controller);

  auto& buttons = controller.getButtonsMut();
  buttons.update();
  auto& sliders = controller.getSlidersMut();
  //sliders.set(ACCELERATION_POTENTIOMETER, ioState.getPotentiometerValue(ACCELERATION_POTENTIOMETER));
  //sliders.set(REGENERATION_POTENTIOMETER, ioState.getPotentiometerValue(REGENERATION_POTENTIOMETER));
  int oldVal = sliders.get(0);
  //int val = min(max(analogRead(34) - 1000, 0) / 3, MAX_POT_VALUE);
  //sliders.set(0, val);
  //controller.update(millis(), micros());

  //if(millis() - lastDriveCommandTime >= 50) {
    //lastDriveCommandTime = millis();
    SpeedCommand command = controller.motorCommand();
    //sendCan(DriveCommand(command.current, command.velocity));
  //}
}
