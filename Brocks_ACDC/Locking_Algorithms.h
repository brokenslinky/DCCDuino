#include "Vehicle_State.h"

struct LockingAlgorithms {
    static int calculate_lock(VehicleState state) {
        if (state.longitudinal_sensitivity == 0) {
            // Manual mode is accessed by reducing the longitudinal sensitivity to zero.
            state.lockup = 127.0 * state.lateral_sensitivity / 15; // manual mode if rampRate set to zero
        } else {
            state.lockup = sensitivity_algorithm(state);
        }

        if (state.lockup > 127.0) {
            state.lockup = 127.0; // don't exceed 50% duty cycle
        }
        if (state.lockup < 0.0) {
            state.lockup = 0.0; // no negative PWM values
        }

        return state.lockup;
    }

    /**
     * Lock increases linearly with forward or lateral acceleration.
     * Braking has a programmable dead zone.
    **/
    static int sensitivity_algorithm(VehicleState state);

    /**
     * Dead zone and ramp widths for each:
     * forward, braking, and lateral acceleration
    **/
    static int donut_algorithm(VehicleState state);

    /**
     * Lock increases linearly with longitudinal acceleration
     * and with ratio of rotational speed to lateral acceleration / speed
    **/
    static int yaw_rate_reaction(VehicleState state);
};