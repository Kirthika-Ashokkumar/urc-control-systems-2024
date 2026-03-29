#pragma once
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace sjsu::science {
class science_mixer
{
private:
  hal::v5::strong_ptr<hal::pwm16_channel> m_pwm;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;

public:
  science_mixer(hal::v5::strong_ptr<hal::pwm16_channel> p_pwm,
                hal::v5::strong_ptr<hal::steady_clock> p_clock)
    : m_pwm(p_pwm)
    , m_clock(p_clock)
  {
  }

  void constant_velocity(hal::time_duration time);
  void slow_to_constant_velocity(hal::time_duration time);
  
};

}  // namespace sjsu::science
