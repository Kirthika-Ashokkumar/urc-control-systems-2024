
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

  auto& led = *hardware_map.led.value();
  auto& clock = *hardware_map.clock.value();
  auto& console = *hardware_map.console.value();

  hal::print(console, "Starting Application!\n");
  hal::print(console, "Will reset after ~10 seconds\n");

  for (int i = 0; i < 10; i++) {
    // Print message
    hal::print(console, "Hello, World\n");

    // Toggle LED
    led.level(true);
    hal::delay(clock, 500ms);

    led.level(false);
    hal::delay(clock, 500ms);
  }

  hal::print(console, "Resetting!\n");
  hal::delay(clock, 100ms);
  hardware_map.reset();



  // application runs homing
  // stuff to send to mission control:
  // RMD offsets (if MC can't do math then just send states)
  // invalid target state (throw error)
  // system error (no response or something)
  // accelerometer positions? (accelerometer driver speeds)
  // drive speeds (target + actual (motors vs accelerometer))? (cause MC doesn't want to do math)
}
}
