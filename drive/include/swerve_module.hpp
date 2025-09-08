#pragma once

#include "swerve_structs.hpp"
#include "vector2d.hpp"
#include <libhal/servo.hpp>
#include <libhal/units.hpp>
#include <libhal/motor.hpp>
#include <libhal/input_pin.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>


namespace sjsu::drive {

struct swerve_module_settings
{
  vector2d position;
  meters_per_sec max_speed; // only thng that feels need to be intialized at begining
  meters_per_sec_per_sec acceleration; //isnt this dependent on the mc commands
  deg_per_sec turn_speed; //isnt this dependent on the mc commands
  hal::degrees min_angle; // won't know until limit switch mount
  hal::degrees max_angle; // won't know until limit switch mount
};

struct swerve_module_hardware
{
  hal::actuator::rmd_mc_x_v2* steer;
  hal::actuator::rmd_mc_x_v2* propulsion;
  hal::input_pin* limit_switch;
  // drivers::tmag5273* tmag;
};

class swerve_module
{
public:
  swerve_module_settings settings;
  swerve_module_hardware hardware;
  bool reversed;
  hal::degrees homing_offset;
  // vars:
  // steer postion read & control (maybe also contains restrictions) (has homing
  // function) prop vel read & control (maybe also contrans restrictions)
  // settings (positions & maybe constraints of motors if not included)
  // most recent measurements (so it don't send a read command every time)

  // methods:
  // calc target state for vector (closest angle for valid state) 
  // 
  // vector2d calc_module_vector(chassis_velocities p_chassis_velocities);
  // // set target state
  // void set_target_state(swerve_module_state p_target_state);
  // // TODO: move to calc file?
  // swerve_module_state calc_closest_state(vector2d p_module_vector);
  // // optimize freedom
  // swerve_module_state calc_freest_state(vector2d p_module_vector);
  // //in valid range
  // bool can_reach_state(swerve_module_state p_state);

  // // time for transition
  // sec transition_time(swerve_module_state p_end_state);

  // // reads state cache
  // swerve_module_state get_actual_state_cache();
  // // refresh state cache
  // swerve_module_state refresh_actual_state_cache();

  // get actual & target state
  // get actual angle
  // get actual prop vel
  // get target angle
  // get target prop vel
  // set target angle
  // set target prop vel
};
}  // namespace sjsu::drive