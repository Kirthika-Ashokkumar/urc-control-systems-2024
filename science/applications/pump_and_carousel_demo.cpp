#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../include/pump_manager.hpp"
#include "../include/carousel.hpp"

#include <resource_list.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::science {

void application()
{
  // configure drivers
  auto clock = resources::clock();
  auto terminal = resources::console();
  // auto carousel_servo_ptr = resources::carousel_servo();
  // carousel carousel_servo(carousel_servo_ptr);
  // carousel_servo.home();


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
  // int step = 1;

  while (true) {
    hal::print<64>(*terminal, "pumping DI water\n");
    m_pump_manager.pump(pump_manager::pumps::DEIONIZED_WATER, 1000ms);
    hal::print<64>(*terminal, "pumping biuret reagent\n");
    m_pump_manager.pump(pump_manager::pumps::KALLING_REAGENT, 1000ms);
    // carousel_servo.step_move(step);
    // hal::delay(*clock, 500ms );
    // step++;
    // if(step >= 9){
    //   step = 1;
    // }

  }
}
}  // namespace sjsu::science