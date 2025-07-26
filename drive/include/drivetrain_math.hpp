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
  std::array<swerve_module, module_count> p_modules);

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
sec calculate_total_interpolation_time(swerve_module p_module,
                                       swerve_module_state p_end_state);

sec calculate_total_interpolation_time(
  std::array<swerve_module, module_count> p_modules,
  std::array<swerve_module_state, module_count> p_end_states);

// detect interpolation conflicts (ivalid interpolation how calc?)

// scale down speeds
std::array<swerve_module_state, module_count> scale_down_propulsion_speed(
  std::array<swerve_module, module_count> p_modules,
  std::array<swerve_module_state, module_count> p_states);

swerve_module_state interpolate_state(float p_portion,
                                      swerve_module_state p_start_state,
                                      swerve_module_state p_end_state);

std::array<swerve_module_state, module_count> interpolate_states(
  sec p_cycle_time,
  std::array<swerve_module, module_count> p_modules,
  std::array<swerve_module_state, module_count> p_end_states);
}  // namespace sjsu::drive