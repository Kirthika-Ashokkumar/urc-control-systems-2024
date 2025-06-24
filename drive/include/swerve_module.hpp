#pragma once

namespace sjsu::drive {

class swerve_module {
    // vars:
    // steer postion read & control (maybe also contains restrictions) (has homminhg funciton)
    // prop vel read & control (maybe also contrans restrictions)
    // settings (positions & maybe constraints of motors if not included)

    // methods:
    // initialize/ready (homing)

    // calc target state for vector
    // set target state
    // optimize freedom
    // update motors/ run control loop (repeatedly called) (dose not runs interpolation)

    // get actual & target state
    // get actual angle
    // get actual prop vel
    // get target angle
    // get target prop vel
    // set target angle
    // set target prop vel

};
}