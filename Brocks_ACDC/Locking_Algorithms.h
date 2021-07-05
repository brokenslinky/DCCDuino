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

    static int sensitivity_algorithm(VehicleState state);

    static int yaw_rate_reaction(VehicleState state);
};