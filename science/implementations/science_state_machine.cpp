#include "../include/science_state_machine.hpp"
namespace sjsu::science {
    science_state_machine::science_state_machine(hal::actuator::rc_servo p_arm_servo,
                        hal::actuator::rc_servo p_trap_door,
                        hal::actuator::rc_servo p_mixer,
                        hal::actuator::rc_servo p_door,
                        hal::v5::strong_ptr<carousel> p_carousel,
                        hal::v5::strong_ptr<pump_manager> p_pump_manager,
                        hal::v5::strong_ptr<hal::steady_clock> p_clock,
                        hal::v5::strong_ptr<hal::serial> p_terminal){

                        }

    int science_state_machine::get_num_vials(){

    }
    void science_state_machine::turn_on_pump(auto pump, hal::time_duration duration){

    }

    void science_state_machine::run_state_machine(science_states state){
        
    }

}


