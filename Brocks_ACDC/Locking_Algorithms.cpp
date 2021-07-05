#include "Locking_Algorithms.h"
#include <math.h>

int LockingAlgorithms::sensitivity_algorithm(VehicleState state) {
    float longitudinal_friction = 1.6 - 0.1 * (float)state.longitudinal_sensitivity;
    float lateral_friction      = 1.6 - 0.1 * (float)state.lateral_sensitivity;

    // Different logic for braking versus accelerating. A dead zone under light deceleration may be useful.
    float x_squared = 0.0;
    if (state.longitudinal_accel > 0.0) {
        x_squared = state.longitudinal_accel / longitudinal_friction;
    } else if (fabs(state.longitudinal_accel / state.vertical_accel) > state.brake_lock_begin * 0.1) {
        x_squared = (fabs(state.longitudinal_accel) - state.brake_lock_begin * 0.1 * state.vertical_accel) / 
                  (1.0 + state.brake_ramp_width * 0.1);
    }
    x_squared *= x_squared;

    float y_squared = (state.lateral_accel / lateral_friction) * (state.lateral_accel / lateral_friction);

    state.lockup = 127.0 * sqrt(x_squared + y_squared) / fabs(state.vertical_accel); 

    return state.lockup;
}

int LockingAlgorithms::yaw_rate_reaction(VehicleState state) {
    float friction = 1.6 - 0.1 * (float)state.longitudinal_sensitivity;
    float ramp_rate = 1.0 / ( 3.01 - 0.2 * (float)state.lateral_sensitivity);
    float lock_from_acceleration;
    if (state.longitudinal_accel > 0.0) {
      lock_from_acceleration = 1.625 * (state.longitudinal_accel * cos(state.slip) - fabs(state.lateral_accel) * sin(state.slip)) / 
                               (friction * state.vertical_accel) - 1;
    } else if (fabs(state.lateral_accel / state.vertical_accel) > state.brake_lock_begin * 0.1) {
        lock_from_acceleration = (fabs(state.longitudinal_accel / state.vertical_accel) - state.brake_lock_begin * 0.1) / 
                                 (1.0 + state.brake_ramp_width * 0.1);
    }
    float lock_from_yaw_rate = ramp_rate * 0.04559 * fabs(state.yaw_rate) * state.longitudinal_speed / 
                      (fabs(state.lateral_accel) * cos(state.slip) * cos(state.slip) + 
                      state.longitudinal_accel * cos(state.slip) * sin(state.slip));
    state.lockup = 127.0 * (lock_from_acceleration + lock_from_yaw_rate);
    return state.lockup;
}