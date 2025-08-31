#include <libhal-micromod/micromod.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <libhal/can.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>


#include "../applications/application.hpp"
#include "../include/swerve_structs.hpp"
#include "../include/swerve_module.hpp"


namespace sjsu::drive {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;

  hal::micromod::v1::initialize_platform();
  static auto& terminal = hal::micromod::v1::console(hal::buffer<1024>);
  static auto& counter = hal::micromod::v1::uptime_clock();

  hal::print<1028>(terminal, "Created input pin\n");
  static hal::stm32f1::input_pin fl_pin_1('B', 1);  // 60 spi1_sck
  static hal::stm32f1::input_pin fr_pin_2('B', 0);  // 62 spi1_copi
  static hal::stm32f1::input_pin bl_pin_3('A', 7);  // 64 spi1_cipo
  static hal::stm32f1::input_pin br_pin_4('B', 0);  // 34 A0

  static hal::can_transceiver* can_transceiver;
  static hal::can_bus_manager* bus_man;
  static hal::can_identifier_filter* idf0 = &hal::micromod::v1::can_identifier_filter0();
  static hal::can_identifier_filter* idf1 = &hal::micromod::v1::can_identifier_filter1();
  static hal::can_identifier_filter* idf2 = &hal::micromod::v1::can_identifier_filter2();
  static hal::can_identifier_filter* idf3 = &hal::micromod::v1::can_identifier_filter3();
  static hal::can_identifier_filter* idf4 = &hal::micromod::v1::can_identifier_filter4();
  static hal::can_identifier_filter* idf5 = &hal::micromod::v1::can_identifier_filter5();
  static hal::can_identifier_filter* idf6 = &hal::micromod::v1::can_identifier_filter6();
  static hal::can_identifier_filter* idf7 = &hal::micromod::v1::can_identifier_filter4();

  hal::print<1028>(terminal, "can initialized\n");
  bus_man->baud_rate(1.0_MHz);

  static swerve_module front_left_wheel;
  static hal::actuator::rmd_mc_x_v2* mc_x_front_left_steer;
  try{
    static hal::actuator::rmd_mc_x_v2 temp1(
      *can_transceiver,
      *idf0,
      counter,
      36.0f,
      0x14c);
    mc_x_front_left_steer = &temp1;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  static hal::actuator::rmd_mc_x_v2* mc_x_front_left_prop;
  try{
    static hal::actuator::rmd_mc_x_v2 temp2(
      *can_transceiver,
      *idf1,
      counter,
      36.0f,
      0x150);
    mc_x_front_left_steer = &temp2;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  front_left_wheel.hardware.steer = mc_x_front_left_steer;
  front_left_wheel.hardware.propulsion = mc_x_front_left_prop;
  front_left_wheel.hardware.limit_switch = &fl_pin_1;
  front_left_wheel.reversed = false;

  static swerve_module front_right_wheel;
  static hal::actuator::rmd_mc_x_v2* mc_x_front_right_steer;
  try{
    static hal::actuator::rmd_mc_x_v2 temp3(
      *can_transceiver,
      *idf2,
      counter,
      36.0f,
      0x143);
    mc_x_front_left_steer = &temp3;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  static hal::actuator::rmd_mc_x_v2* mc_x_front_right_prop;
  try{
    static hal::actuator::rmd_mc_x_v2 temp4(
      *can_transceiver,
      *idf3,
      counter,
      36.0f,
      0x151);
    mc_x_front_left_steer = &temp4;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  front_right_wheel.hardware.steer = mc_x_front_right_steer;
  front_right_wheel.hardware.propulsion = mc_x_front_right_prop;
  front_right_wheel.hardware.limit_switch = &fr_pin_2;
  front_right_wheel.reversed = false;

  static swerve_module back_left_wheel;
  static hal::actuator::rmd_mc_x_v2* mc_x_back_left_wheel;
  try{
    static hal::actuator::rmd_mc_x_v2 temp5(
      *can_transceiver,
      *idf4,
      counter,
      36.0f,
      0x14c);
    mc_x_front_left_steer = &temp5;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  static hal::actuator::rmd_mc_x_v2* mc_x_back_left_prop;
  try{
    static hal::actuator::rmd_mc_x_v2 temp6(
      *can_transceiver,
      *idf5,
      counter,
      36.0f,
      0x150);
    mc_x_front_left_steer = &temp6;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  back_left_wheel.hardware.steer = mc_x_back_left_wheel;
  back_left_wheel.hardware.propulsion = mc_x_back_left_prop;
  back_left_wheel.hardware.limit_switch = &bl_pin_3;
  back_left_wheel.reversed = true;


  static swerve_module back_right_wheel;
  static hal::actuator::rmd_mc_x_v2* mc_x_back_right_wheel;
  try{
    static hal::actuator::rmd_mc_x_v2 temp7(
      *can_transceiver,
      *idf6,
      counter,
      36.0f,
      0x14c);
    mc_x_front_left_steer = &temp7;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  static hal::actuator::rmd_mc_x_v2* mc_x_back_right_prop;
  try{
    static hal::actuator::rmd_mc_x_v2 temp8(
      *can_transceiver,
      *idf7,
      counter,
      36.0f,
      0x150);
    mc_x_front_left_steer = &temp8;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  back_right_wheel.hardware.steer = mc_x_back_right_wheel;
  back_right_wheel.hardware.propulsion = mc_x_back_right_prop;
  back_right_wheel.hardware.limit_switch = &br_pin_4;
  back_right_wheel.reversed = true;

  static std::array<swerve_module, 4> swerve_modules_span = {
    front_left_wheel,
    front_right_wheel,
    back_left_wheel,
    back_right_wheel
  };

  return {
    .led = &hal::micromod::v1::led(), 
    .console = &terminal,
    .clock = &counter,
    .swerve_modules = swerve_modules_span,
    .reset = +[]() { hal::micromod::v1::reset(); },
  };
}
}
