#include <libhal-actuator/rc_servo.hpp>
#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../include/carousel.hpp"
#include "../resource_list.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::science {
void testing(carousel& carousel_servo)
{
  auto clock = resources::clock();
  auto terminal = resources::console();
  // while(true){
  //     hal::delay(*clock, 1000ms);
  //     try{
  //         carousel_servo.step_move();
  //     }
  //     catch(const char* err) {
  //         hal::print<16>(*terminal, "%s\n", err);
  //         break;
  //     }
  //     hal::print(*terminal, "Moved Servo forward one turn\n");
  //     hal::delay(*clock, 100ms);
  // }
  // while(true){
  //     hal::delay(*clock, 1000ms);
  //     try{
  //         carousel_servo.step_backward();
  //     }
  //     catch(const char* err) {
  //         hal::print<16>(*terminal, "%s\n", err);
  //         break;
  //     }
  //     hal::print(*terminal, "Moved Servo backward one turn\n");
  //     hal::delay(*clock, 100ms);
  // }
  while (true) {
    hal::delay(*clock, 2000ms);
    try {
      carousel_servo.step_move(1);
    } catch (const char* err) {
      hal::print<16>(*terminal, "%s\n", err);
      break;
    }
    hal::print(*terminal, "Moved Servo forward 3 turns\n");
  }
  while (true) {
    hal::delay(*clock, 2000ms);
    try {
      carousel_servo.step_backward(1);
    } catch (const char* err) {
      hal::print<16>(*terminal, "%s\n", err);
      break;
    }
    hal::print(*terminal, "Moved Servo backward 3 turns\n");
  }
}

void application()
{
  auto clock = resources::clock();
  auto terminal = resources::console();
  auto carousel_servo_ptr = resources::carousel_servo();

  hal::print(*terminal, "Hello, Program Started\n");
  hal::delay(*clock, 100ms);
  carousel carousel_servo(carousel_servo_ptr);

  carousel_servo.home();
  hal::print(*terminal, "Moved Servo to Start\n");
  hal::delay(*clock, 1000ms);

  while (true) {
    // testing(carousel_servo);
    carousel_servo.step_move(1);
    hal::print(*terminal, "Moved forward 1\n");
    hal::delay(*clock, 5000ms);

    carousel_servo.step_backward(1);
    hal::print(*terminal, "Moved backward 1\n");
    hal::delay(*clock, 5000ms);

  }
}
}  // namespace sjsu::science
