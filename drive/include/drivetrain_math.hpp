#include "swerve_module.hpp"
#include "vector2d.hpp"
#include <cmath>
#include <libhal/units.hpp>
#include <span>
namespace sjsu::drive {

// chassis speed to states
void chassis_velocities_to_module_vectors(
  chassis_velocities p_chassis_velocities,
  std::span<swerve_module> p_modules,
  std::span<vector2d> p_vector_buffer);
// module state validity score (0 is complete match)
float module_validity_strain_score(std::span<swerve_module> p_modules,
                                   std::span<vector2d> p_vectors);
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
  swerve_module_state p_end_state);
hal::time_duration calculate_total_interpolation_time(
  std::span<swerve_module> p_modules,
  std::span<swerve_module_state> p_end_states);

// detect interpolation conflicts (ivalid interpolation how calc?)

// scale down speeds
void scale_down_propulsion_speed(std::span<swerve_module> p_modules,
                                 std::span<swerve_module_state> p_states);
// interpolate state (by % and/or time) - time can't be done with just 2 states
// since we would have to sync all the modules 
// interpolate states (by % and/ortime) - why would
// percent be a thing?
swerve_module_state interpolate_state(float p_percent, swerve_module_state p_start_state, swerve_module_state p_end_state);
void interpolate_states(hal::time_duration p_cycle_time,
                        std::span<swerve_module> p_modules,
                        std::span<swerve_module_state> p_end_states,
  std::span<swerve_module_state> p_target_states_buffer);
}  // namespace sjsu::drive