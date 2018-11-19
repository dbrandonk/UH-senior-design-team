#include "Sub.h"

// stabilize_init - initialise stabilize controller
bool Sub::motor_direct_init()
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    //change  if we are controlling the sub motors directly
    motors.direct_motor_control_mode(true);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::motor_direct_run()
{
     // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    unsigned short* rc_pwm_input;
    unsigned short rc_input_values[8];

    for (int i=0; i<8; i++) {
        rc_input_values[i] = RC_Channels::rc_channel(i)->read();

    }

    rc_pwm_input = rc_input_values;

    motors.direct_motor_control(rc_pwm_input);

}
