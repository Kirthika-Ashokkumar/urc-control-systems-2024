#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "../include/carousel.hpp"
#include "../include/pump_manager.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;
#include <resource_list.hpp>

namespace sjsu::science {

void application()
{
  // configure drivers
  auto clock = resources::clock();
  auto terminal = resources::console();

  auto deionized_pump = resources::deionized_water_pump();
  hal::print(*terminal, "DI pump\n");
  auto benedict_pump = resources::benedict_reagent_pump();
  hal::print(*terminal, "BEN pump\n");
  auto biuret_pump = resources::biuret_reagent_pump();
  hal::print(*terminal, "BIUR pump\n");
  auto kalling_pump = resources::kalling_reagent_pump();
  hal::print(*terminal, "KALL pump\n");

  auto m_pump_manager = pump_manager(
    clock, deionized_pump, benedict_pump, biuret_pump, kalling_pump);
  auto trap_door_servo = resources::trap_door_servo();
  auto gyro_cup_servo = resources::gyro_cup_servo();
  auto arm_belt_servo = resources::arm_belt_servo();
  auto carousel_servo_ptr = resources::carousel_servo();

  carousel carousel_servo(carousel_servo_ptr);
  carousel_servo.home();

  try {

    while (true) {

      //   arm_belt_servo->position(5);
      //   hal::delay(*clock, 3000ms);
      //   hal::print(*terminal, "cup start moving\n");

      //   arm_belt_servo->position(-45);
      //   hal::delay(*clock, 700ms);
      //   hal::print(*terminal, "cup stop moving\n");

      //   arm_belt_servo->position(5);
      //   hal::delay(*clock, 3000ms);
      //   hal::print(*terminal, "cup start moving\n");

      //   arm_belt_servo->position(60);
      //   hal::delay(*clock, 1500ms);
      //   hal::print(*terminal, "cup stop moving\n");

      // close trap door
      trap_door_servo->position(90);
      hal::delay(*clock, 1000ms);
      hal::print(*terminal, "close trap door\n");

      // move out cup
      gyro_cup_servo->position(220);
      hal::delay(*clock, 1000ms);
      hal::print(*terminal, "move out cup\n");

      // move cup in
      gyro_cup_servo->position(270);
      hal::delay(*clock, 1000ms);
      hal::print(*terminal, "move cup in\n");

      // open trap door
      trap_door_servo->position(270);
      hal::delay(*clock, 1000ms);
      hal::print(*terminal, "open trap door\n");

      // close trap door
      trap_door_servo->position(90);
      hal::delay(*clock, 1000ms);
      hal::print(*terminal, "close trap door\n");

      m_pump_manager.pump(pump_manager::pumps::DEIONIZED_WATER, 1000ms);
      hal::print(*terminal, "Running DEIONIZED_WATER pump\n");

      m_pump_manager.pump(pump_manager::pumps::BENEDICT_REAGENT, 1000ms);
      hal::print(*terminal, "Running BENEDICT_REAGENT pump\n");

      m_pump_manager.pump(pump_manager::pumps::BIURET_REAGENT, 1000ms);
      hal::print(*terminal, "Running BIURET_REAGENT pump\n");

      m_pump_manager.pump(pump_manager::pumps::KALLING_REAGENT, 1000ms);
      hal::print(*terminal, "Running KALLING_REAGENT pump\n");
      hal::delay(*clock, 1000ms);

      carousel_servo.step_move(1);
      hal::print(*terminal, "Moved forward 1\n");
      hal::delay(*clock, 1000ms);

      carousel_servo.step_backward(1);
      hal::print(*terminal, "Moved backward 1\n");
      hal::delay(*clock, 1000ms);

      while (true) {
        hal::delay(*clock, 2000ms);
        try {
          carousel_servo.step_move(3);
        } catch (const char* err) {
          hal::print<16>(*terminal, "%s\n", err);
          break;
        }
        hal::print(*terminal, "Moved Servo forward 3 turns\n");
      }

      while (true) {
        hal::delay(*clock, 2000ms);
        try {
          carousel_servo.step_backward(3);
        } catch (const char* err) {
          hal::print<16>(*terminal, "%s\n", err);
          break;
        }
        hal::print(*terminal, "Moved Servo backward 3 turns\n");
      }
    }
  } catch (hal::exception e) {
    hal::print<64>(*terminal, "error code: %d", e.error_code());
  }
}
}  // namespace sjsu::science