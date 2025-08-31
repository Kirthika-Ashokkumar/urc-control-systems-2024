
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/steady_clock.hpp>

#include "./application.hpp"


namespace sjsu::drive {
void application(hardware_map_t& hardware_map)
{
  using namespace std::chrono_literals;

  // auto& led = *hardware_map.led.value();
  auto& clock = *hardware_map.clock.value();
  auto& console = *hardware_map.console.value();
  auto& swerve_module = *hardware_map.swerve_modules;

  hal::print(console, "Starting Application!\n");
  hal::print(console, "Will reset after ~10 seconds\n");




  // application runs homing
  // stuff to send to mission control:
  // RMD offsets (if MC can't do math then just send states)
  // invalid target state (throw error)
  // system error (no response or something)
  // accelerometer positions? (accelerometer driver speeds)
  // drive speeds (target + actual (motors vs accelerometer))? (cause MC doesn't want to do math)
}
}
