#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

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
  
  auto deionized_pump  = resources::deionized_water_pump();
  hal::print(*terminal, "DI pump\n");
  auto benedict_pump = resources::benedict_reagent_pump();
  hal::print(*terminal, "BEN pump\n");
  auto biuret_pump = resources::biuret_reagent_pump();
  hal::print(*terminal, "BIUR pump\n");
  auto kalling_pump = resources::kalling_reagent_pump();
  hal::print(*terminal, "KALL pump\n");
  
  // TODO: replace nullptrs once hardwaremap has been updated
  auto m_pump_manager = pump_manager(clock,
                                    deionized_pump,
                                    benedict_pump,
                                    biuret_pump,
                                    kalling_pump);

  while (true) {
    hal::print(*terminal, "hello i am working");
    m_pump_manager.pump(pump_manager::pumps::DEIONIZED_WATER, 1000ms);
    m_pump_manager.pump(pump_manager::pumps::BENEDICT_REAGENT, 1000ms);
    m_pump_manager.pump(pump_manager::pumps::BIURET_REAGENT, 1000ms);
    m_pump_manager.pump(pump_manager::pumps::KALLING_REAGENT, 1000ms);

  }
}
}  // namespace sjsu::science