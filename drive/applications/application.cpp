#include "./application.hpp"

#include "drive_configuration_updater.hpp"
#include "homing.hpp"
#include "settings.hpp"
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/units.hpp>

namespace sjsu::drive {

void application(hardware_map_t& hardware_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *hardware_map.clock.value();
  auto& console = *hardware_map.terminal.value();
  auto& can_transceiver = *hardware_map.can_transceiver.value();
  // auto& can_bus_manager = *hardware_map.can_bus_manager.value();
  // auto& can_identifier_filter = *hardware_map.can_identifier_filter.value();

  hal::can_message_finder spin_reader(can_transceiver, 0x101);
  hal::can_message_finder drive_reader(can_transceiver, 0x102);
  hal::can_message_finder translate_reader(can_transceiver, 0x103);
  hal::can_message_finder speed_reader(can_transceiver, 0x104);
  hal::can_message_finder homing_reader(can_transceiver, 0x105);

  // const hal::u8 system_reset = 0x76;
  hal::print(console, "created things that we need.\n");

  auto& steering_modules = *hardware_map.steering_modules;
  auto& start_wheel_settings = *hardware_map.start_wheel_setting_span;
  
  home(steering_modules, start_wheel_settings, clock, console);

  while (false) {
    try {
      std::optional<hal::can_message> msg = homing_reader.find();
      if (msg) {
        hal::print(console, "found message\n");
        home(steering_modules, start_wheel_settings, clock, console);

        hal::print(console, "Done homing\n");
      }
      hal::print<128>(console,
                      "Circular Buffer Size: %d\n",
                      can_transceiver.receive_cursor());
    } catch (hal::timed_out const&) {
      hal::print(
        console,
        "hal::timed_out exception! which means that the device did not "
        "respond. Moving to the next device address in the list.\n");
    } catch (hal::resource_unavailable_try_again const& p_error) {
      hal::print(console, "hal::resource_unavailable_try_again\n");
      if (p_error.instance() == &can_transceiver) {
        hal::print(
          console,
          "\n"
          "device on the bus. It appears as if the peripheral is not connected "
          "to a can network. This can happen if the baud rate is incorrect, "
          "the CAN transceiver is not functioning, or the devices on the bus "
          "are not responding."
          "\n"
          "Calling terminate!"
          "\n"
          "Consider powering down the system and checking all of your "
          "connections before restarting the application.");
        std::terminate();
      }
      // otherwise keep trying with other addresses
    } catch (...) {
      hal::print(console, "Unknown exception caught in (...) block\n");
      throw;  // see if anyone else can handle the exception
    }

    // address_offset = (address_offset + 1) % 16;
    hal::delay(clock, 1s);
  }
}

}  // namespace sjsu::drive
