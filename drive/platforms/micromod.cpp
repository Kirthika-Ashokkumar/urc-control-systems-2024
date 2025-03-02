// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/motor.hpp>
#include <span>
#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

#include "ackermann_steering.hpp"
#include <array>
#include <libhal-micromod/micromod.hpp>
#include <libhal-util/units.hpp>

#include "../applications/application.hpp"

#include "settings.hpp"
#include "steering_module.hpp"
#include "vector2.hpp"

namespace sjsu::drive {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  static auto& counter = hal::micromod::v1::uptime_clock();

  static auto& terminal = hal::micromod::v1::console(hal::buffer<1024>);

  // static auto& fl_pin_1 = hal::micromod::v1::input_g0();
  // static auto& fr_pin_2 = hal::micromod::v1::input_g1();
  static auto& bl_pin_3 = hal::micromod::v1::input_g0();
  // static auto& br_pin_4 = hal::micromod::v1::input_g3();

  // static std::array<vector2, 3> wheel_locations = {
  //   vector2::from_bearing(1, -60 * std::numbers::pi / 180),
  //   vector2::from_bearing(1, 60 * std::numbers::pi / 180),
  //   vector2::from_bearing(1, std::numbers::pi),
  // };

  // static std::array<wheel_setting, wheel_locations.size()> wheel_settings;

  // Note: Maximum speed is not used
  // static ackermann_steering steering(wheel_locations, wheel_settings, 2, 2);

  // WARINING: THESE MODULES HAVE NOT BEEN INITIALIZED
  // static std::array<steering_module, 1> modules;

  static hal::can_transceiver* can_transceiver;
  static hal::can_bus_manager* bus_man;
  static hal::can_identifier_filter* idf;

  static std::array<hal::can_message, 8> receive_buffer{};
  can_transceiver = &hal::micromod::v1::can_transceiver(receive_buffer);
  
  bus_man = &hal::micromod::v1::can_bus_manager();
  idf = &hal::micromod::v1::can_identifier_filter0();
  idf->allow(0x105);
  hal::print<1028>(terminal, "can initialized\n");
  bus_man->baud_rate(1.0_MHz);
  // idf->allow(0x248);
  // static std::array<start_wheel_setting, 4> start_wheel_setting_arr = {
  //   front_left_wheel_setting,
  //   front_right_wheel_setting,
  //   back_left_wheel_setting,
  //   back_right_wheel_setting
  // };

  // static std::span<start_wheel_setting,4> start_wheel_setting_span =
  // start_wheel_setting_arr;

  static std::array<start_wheel_setting, 1> start_wheel_setting_arr = {
    back_left_wheel_setting
  };

  static std::span<start_wheel_setting, 1> start_wheel_setting_span =
    start_wheel_setting_arr;

  hal::print<1028>(terminal, "Stetting struct intialized\n");

  // static hal::actuator::rmd_mc_x_v2 mc_x_front_left_prop(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[0].geer_ratio,
  //   start_wheel_setting_arr[0].prop_id);
  // static auto front_left_prop =
  //   mc_x_front_left_prop.acquire_motor(start_wheel_setting_arr[0].max_speed);
  // static hal::actuator::rmd_mc_x_v2 mc_x_front_left_steer(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[0].geer_ratio,
  //   start_wheel_setting_arr[0].steer_id);
  // static steering_module front_left_leg = {
  //   .steer = &mc_x_front_left_steer,
  //   .propulsion = &front_left_prop,
  //   .limit_switch = &fl_pin_1,
  // };

  // static hal::actuator::rmd_mc_x_v2 mc_x_front_right_prop(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[1].geer_ratio,
  //   start_wheel_setting_arr[1].prop_id);
  // static auto front_right_prop =
  //   mc_x_front_right_prop.acquire_motor(start_wheel_setting_arr[1].max_speed);
  // static hal::actuator::rmd_mc_x_v2 mc_x_front_right_steer(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[1].geer_ratio,
  //   start_wheel_setting_arr[1].steer_id);
  // static steering_module front_right_leg = {
  //   .steer = &mc_x_front_right_steer,
  //   // .propulsion = nullptr
  //   .propulsion = &front_right_prop,
  //   .limit_switch= &fr_pin_2,
  // };

  // static hal::actuator::rmd_mc_x_v2 mc_x_back_left_prop(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[2].geer_ratio,
  //   start_wheel_setting_arr[2].prop_id);

  // static auto back_left_prop =
  //   mc_x_back_left_prop.acquire_motor(start_wheel_setting_arr[2].max_speed);
  static hal::actuator::rmd_mc_x_v2* mc_x;
  try {
    static hal::actuator::rmd_mc_x_v2 mc_x_back_left_steer(
      *can_transceiver,
      *idf,
      counter,
      start_wheel_setting_arr[0].geer_ratio,
      start_wheel_setting_arr[0].steer_id);
    mc_x = &mc_x_back_left_steer;
    hal::print<1028>(terminal, "RMD created\n");

  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "RMD did not get created\n");
  }

  static steering_module back_left_leg = {
    .steer = mc_x,
    // .propulsion = &back_left_prop,
    .propulsion = nullptr,
    .limit_switch = &bl_pin_3,
  };

  // hal::print<1028>(terminal, "Steer rmd intialized\n");

  // static hal::actuator::rmd_mc_x_v2 mc_x_back_right_prop(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[3].geer_ratio,
  //   start_wheel_setting_arr[3].prop_id);
  // static auto back_right_prop =
  //   mc_x_back_left_prop.acquire_motor(start_wheel_setting_arr[3].max_speed);
  // static hal::actuator::rmd_mc_x_v2 mc_x_back_right_steer(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[3].geer_ratio,
  //   start_wheel_setting_arr[3].steer_id);
  // static steering_module back_right_leg = {
  //   .steer = &mc_x_back_right_steer,
  //   .propulsion = &back_right_prop,
  //   .limit_switch = &br_pin_4,
  // };

  // static std::array<steering_module, 4> steering_modules_arr = {
  //   front_left_leg, front_right_leg, back_left_leg, back_right_leg
  // };
  // static std::span<steering_module, 4> steering_modules_span =
  //   steering_modules_arr;

  static std::array<steering_module, 1> steering_modules_arr = {
    back_left_leg
  };
  static std::span<steering_module, 1> steering_modules_span =
    steering_modules_arr;

  return hardware_map_t{
    .clock = &counter,
    .terminal = &terminal,
    .can_transceiver = can_transceiver,
    .can_bus_manager = bus_man,
    .can_identifier_filter = idf,
    .steering_modules = steering_modules_span,
    .start_wheel_setting_span = start_wheel_setting_span,
    .reset = []() { hal::cortex_m::reset(); }
  };
}
}  // namespace sjsu::drive
