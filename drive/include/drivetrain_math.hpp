#include "swerve_module.hpp"
#include "vector2d.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <libhal/units.hpp>
#include <numbers>
#include <sys/types.h>
namespace sjsu::drive {

using namespace std::chrono_literals;

// chassis speed to states
std::array<vector2d, module_count> chassis_velocities_to_module_vectors(
  chassis_velocities p_chassis_velocities,
  std::array<swerve_module, module_count> p_modules)
{
  std::array<vector2d, module_count> vectors;
  //  convert rotaiton speed to radians
  float rotational_vel_radians_per_sec =
    p_chassis_velocities.rotational_vel * std::numbers::pi / 180;
  for (uint i = 0; i < vectors.size(); i++) {
    // translation vector is the same
    vector2d transition = p_chassis_velocities.translation;
    // rotation position vector by 90 degrees is the vector for 1 rad per sec
    vector2d rotation = rotational_vel_radians_per_sec *
                        vector2d::rotate_90_ccw(p_modules[i].settings.position);

    vectors[i] = transition + rotation;
  }
  return vectors;
}

// module state validity score (0 is complete match)
float module_validity_strain_score(
  std::array<swerve_module, module_count> p_modules,
  std::array<vector2d, module_count> p_vectors);
// {
//     vector2d transition(0,0);
//     for (auto& v : p_vectors) {
//         transition =  transition-(v/p_vectors.size());
//     }

//     return 0.0;
// }

// calculate freest state
swerve_module_state calculate_freest_state(swerve_module p_module,
                                           vector2d p_target_vector);
// calc closest equivalent
swerve_module_state calculate_closest_state(swerve_module p_module,
                                            vector2d p_target_vector);
// calc interpolation time (single module and/or all)
hal::time_duration calculate_total_interpolation_time(
  swerve_module p_module,
  swerve_module_state p_end_state)
{
  // using a relatively primitive calculation for now

  // calc speed interpolation time
  meters_per_sec speed_diff =
    fabsf(p_module.get_actual_state_cache().propulsion_velocity -
          p_end_state.propulsion_velocity);

  hal::time_duration propulsion_transition_time =
    1s;  // TODO: make more readable/make 1 line?
  propulsion_transition_time *= speed_diff / p_module.settings.acceleration;

  // hal::time_duration test = 1s * (speed_diff /
  // p_module.settings.max_acceleration);
  hal::degrees angle_diff = fabsf(
    p_module.get_actual_state_cache().steer_angle - p_end_state.steer_angle);
  hal::time_duration steer_transition_time = 1s;
  steer_transition_time *= angle_diff / p_module.settings.turn_speed;

  if (propulsion_transition_time > steer_transition_time) {
    return propulsion_transition_time;
  } else {
    return steer_transition_time;
  }
}

hal::time_duration calculate_total_interpolation_time(
  std::array<swerve_module, module_count> p_modules,
  std::array<swerve_module_state, module_count> p_end_states)
{
  hal::time_duration max_time = 0s;
  for (uint i = 0; i < p_modules.size(); i++) {
    hal::time_duration time =
      calculate_total_interpolation_time(p_modules[i], p_end_states[i]);
    if (time > max_time) {
      max_time = time;
    }
  }
  return max_time;
}

// detect interpolation conflicts (ivalid interpolation how calc?)

// scale down speeds
std::array<swerve_module_state, module_count> scale_down_propulsion_speed(
  std::array<swerve_module, module_count> p_modules,
  std::array<swerve_module_state, module_count> p_states)
{
  float percent = 1.0;
  for (uint i=0; i < p_modules.size(); i++) {
    if(fabsf(p_states[i].propulsion_velocity*percent) > p_modules[i].settings.max_speed) {
      percent = p_modules[i].settings.max_speed / fabsf(p_states[i].propulsion_velocity);
    }
  }
  std::array<swerve_module_state, module_count> scale_down_states;
  for (uint i=0; i< scale_down_states.size(); i++) {
    scale_down_states[i] = {
      .steer_angle = p_states[i].steer_angle,
      .propulsion_velocity = p_states[i].propulsion_velocity * percent
    };
  }
  return scale_down_states;
}
// interpolate state (by % and/or time) - time can't be done with just 2 states
// since we would have to sync all the modules
// interpolate states (by % and/ortime) - why would
// percent be a thing?
swerve_module_state interpolate_state(float p_percent,
                                      swerve_module_state p_start_state,
                                      swerve_module_state p_end_state);
std::array<swerve_module_state, module_count> interpolate_states(
  hal::time_duration p_cycle_time,
  std::array<swerve_module, module_count> p_modules,
  std::array<swerve_module_state, module_count> p_end_states)
{
  std::array<swerve_module_state, module_count> interpolated_states;
  float percent =
    p_cycle_time /
    calculate_total_interpolation_time(
      p_modules, p_end_states);  // TODO: check this is not integer division
  for (uint i = 0; i < p_modules.size(); i++) {
    interpolated_states[i] = interpolate_state(
      percent, p_modules[i].get_actual_state_cache(), p_end_states[i]);
  }
  return interpolated_states;
}
}  // namespace sjsu::drive