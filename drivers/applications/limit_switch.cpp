#include "../hardware_map.hpp"
#include "../include/tmag5273.hpp"
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {

void application(application_framework& p_framework)
{
  auto& clock = *p_framework.steady_clock;
  auto& terminal = *p_framework.terminal;
  auto& limit_switch = *p_framework.in_pin0;

  while (true) {
    int boolean  = limit_switch.level() ? 1 : 0;
    hal::print<64>(terminal, "Value: %d\n", boolean);
    hal::delay(clock, 500ms);
  }
}
}  // namespace sjsu::drivers