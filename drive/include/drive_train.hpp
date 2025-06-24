#pragma once
namespace sjsu::drive {
class drivetrain {
    //store steer modules
    // drive velocity to set state
    // update (run control loop)
    // scale vector down to vector and keep swerve in safe states (how pass in module updates)
    // settings
        // do we interpolate
        // do we flip automatically when necessary
        // do we interpolate until cap or flip immediately
        // when hit cap keep moving?
    //dose all interpolation
    // stuck (invalid state)
    // assume all are homed
};

}