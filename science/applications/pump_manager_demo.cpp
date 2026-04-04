#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../include/pump_manager.hpp"
#include <resource_list.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::science {

void application()
{
  // configure drivers
  auto clock = resources::clock();
  auto terminal = resources::console();

  auto kalling_reagent_pump = resources::kalling_reagent_pump();
  hal::print<64>(*terminal, "hello i getting past the kalling reagent\n");

  auto benedict_reagent_pump = resources::benedict_reagent_pump();
  hal::print<64>(*terminal, "hello i getting past the benedict reagent\n");

  auto deionized_water_pump = resources::deionized_water_pump();
  hal::print<64>(*terminal, "hello i getting past the DI pump\n");

  auto biuret_reagent_pump = resources::biuret_reagent_pump();
  hal::print<64>(*terminal, "hello i getting past the buriet reagent\n");

  // TODO: replace nullptrs once hardwaremap has been updated
  auto m_pump_manager = pump_manager(clock,
                                     deionized_water_pump,
                                     benedict_reagent_pump,
                                     biuret_reagent_pump,
                                     kalling_reagent_pump);
  hal::print<64>(*terminal, "hello i getting past the pump manager\n");

  while (true) {
    hal::print<64>(*terminal, "hello i am working\n");
    m_pump_manager.pump(pump_manager::pumps::DEIONIZED_WATER, 1000ms);
  }
}
}  // namespace sjsu::science