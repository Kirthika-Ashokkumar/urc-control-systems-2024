#pragma once

#include "swerve_structs.hpp"
#include "vector2d.hpp"
#include <libhal/servo.hpp>
#include <libhal/units.hpp>

namespace sjsu::drive {

struct swerve_module_settings
{
  vector2d position;
  meters_per_sec max_speed;
  meters_per_sec_per_sec acceleration;
  deg_per_sec turn_speed;
  hal::degrees min_angle;
  hal::degrees max_angle;
};

class swerve_module
{
public:
  swerve_module_settings settings;
  // vars:
  // steer postion read & control (maybe also contains restrictions) (has homing
  // function) prop vel read & control (maybe also contrans restrictions)
  // settings (positions & maybe constraints of motors if not included)
  // most recent measurements (so it don't send a read command every time)

  // methods:
  // calc target state for vector (closest angle for valid state)
  vector2d calc_module_vector(chassis_velocities p_chassis_velocities);
  // set target state
  void set_target_state(swerve_module_state p_target_state);
  // TODO: move to calc file?
  swerve_module_state calc_closest_state(vector2d p_module_vector);
  // optimize freedom
  swerve_module_state calc_freest_state(vector2d p_module_vector);
  bool can_reach_state(swerve_module_state p_state);

  // time for transition
  sec transition_time(swerve_module_state p_end_state);

  // reads state cache
  swerve_module_state get_actual_state_cache();
  // refresh state cache
  swerve_module_state refresh_actual_state_cache();

  // get actual & target state
  // get actual angle
  // get actual prop vel
  // get target angle
  // get target prop vel
  // set target angle
  // set target prop vel
};
}  // namespace sjsu::drive