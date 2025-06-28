#pragma once
#include "swerve_module.hpp"
namespace sjsu::drive {

struct drive_settings;
enum class drive_status;
class drivetrain
{
  // assume all are homed motors are already homed (will be separate thing to
  // home)
public:
  // store steer modules
  //  drive velocity to set state
  void set_target_state(chassis_velocities p_target_state);

  // update (run control loop)
  // scale vector down to vector and keep swerve in safe states (how pass in
  // module updates)
  void periodic();
  // refresh readings
  void refresh_telemetry();

  // so mission control known we hit a block (instead of int use enum)
  drive_status get_drive_status();
  // can't interpolate (and we don't have perms to auto interpolate)
  // module states became unsafe (how do we get out)

  // settings
  void set_drive_settings(drive_settings p_drive_settings);
  // do we interpolate
  // do we flip automatically when necessary
  // do we interpolate until cap or flip immediately
  // when hit cap keep moving?
  // max interpolation angle? (put in module)
  // what do if interpolation not correct (0 vector)

  // dose all interpolation
  //  stuck (invalid state)
};

}  // namespace sjsu::drive