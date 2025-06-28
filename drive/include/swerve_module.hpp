#pragma once

#include "swerve_structs.hpp"
#include <libhal/servo.hpp>
#include <libhal/units.hpp>

namespace sjsu::drive {
class swerve_module {
    // vars:
    // steer postion read & control (maybe also contains restrictions) (has homing function)
    // prop vel read & control (maybe also contrans restrictions)
    // settings (positions & maybe constraints of motors if not included)
    // most recent measurements (so it don't send a read command every time)

    // methods:
    vector2d calc_module_vector(chassis_velocities p_chassis_velocities);
    // calc target state for vector (closest angle for valid state)
    swerve_module_state calc_closest_state(vector2d p_module_vector);
    // set target state
    void set_target_state(swerve_module_state p_target_state);
    // optimize freedom
    swerve_module_state calc_freest_state(vector2d p_module_vector);
    
    // time for transition
    hal::time_duration transition_time(swerve_module_state p_end_state);

    //reads state cache
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
}