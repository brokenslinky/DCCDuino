#pragma once
#include <stdint.h> // for uint8_t

/** Structure to store to known state of the vehicle **/
static struct VehicleState {
    int     lockup                    = 0;
    float   longitudinal_accel        = 0.0;
    float   lateral_accel             = 0.0;
    float   vertical_accel            = 1.0;
    uint8_t longitudinal_sensitivity  = 0x00;
    uint8_t lateral_sensitivity       = 0x00;
    uint8_t brake_lock_begin          = 0x0f;
    uint8_t brake_ramp_width          = 0x0f;
    uint8_t accel_lock_begin          = 0x0f;
    uint8_t accel_ramp_width          = 0x0f;
    uint8_t lateral_lock_begin        = 0x0f;
    uint8_t lateral_ramp_width        = 0x0f;
    uint8_t manual_mode               = 0x00;
    uint8_t manual_lock_amount        = 0x00;
    float   roll_rate                 = 0.0;
    float   pitch_rate                = 0.0;
    float   yaw_rate                  = 0.0;
    float   longitudinal_speed        = 0.0;
    float   roll_angle                = 0.0;
    float   pitch_angle               = 0.0;
    float   slip                      = 0.0;
} state;