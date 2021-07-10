#include "Display.h"

void Display::init() {
    pinMode(RED_PIN,         OUTPUT);
    pinMode(BLUE_PIN,        OUTPUT);
    pinMode(GREEN_PIN,       OUTPUT);

    lcd_keypad.begin(16, 2);
    lcd_keypad.setBacklight(0x6);

    mode                  = 0; 
    subscreen             = 0;
    printIterationCounter = 0;
}

void Display::print(String line_1, String line_2) {
    lcd_keypad.clear();
    lcd_keypad.setCursor(0, 0);
    lcd_keypad.print(line_1.c_str());
    lcd_keypad.setCursor(0, 1);
    lcd_keypad.print(line_2.c_str());
}

void Display::set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    analogWrite(RED_PIN,   red);
    analogWrite(GREEN_PIN, green);
    analogWrite(BLUE_PIN,  blue);

    const uint8_t threshold = 127;
    uint8_t color = 0x00;
    if (red > threshold) {
        color |= 0x1;
    }
    if (green > threshold) {
        color |= 0x2;
    }
    if (blue > threshold) {
        color |= 0x4;
    }
    lcd_keypad.setBacklight(color);
}

void Display::show_mode() {
    if (mode== DisplayMode::SENSITIVITY_CONFIGS) {
        print("Sensitivity     ",
              "   Configuration");
        // Disable this config screen since we aren't using this algorithm.
        mode++;
        return show_mode();
    } else if (mode == DisplayMode::ACCELERATION_CONFIGS) {
        print("Forward Accel   ",
              "   Configuration");
    } else if (mode == DisplayMode::BRAKING_CONFIGS) {
        print("Braking         ",
              "   Configuration");
    } else if (mode == DisplayMode::LATERAL_CONFIGS) {
        print("Lateral Accel   ",
              "   Configuration");
    } else if (mode == DisplayMode::MANUAL_CONTROL) {
        print("Manual Control  ");
    } else if (mode == DisplayMode::STATS) {
        print("Status          ");
    } else if (mode == DisplayMode::INPUTS) {
        print("Inputs          ");   
    }
    delay_UI(USER_READ_TIME_MILLIS); // Give the user time to read.
}

void Display::update(VehicleState& state) 
{
    // print to LCD
    printIterationCounter++;
    if (printIterationCounter < ITERATIONS_BETWEEN_PRINTS) {
        return;
    }
    printIterationCounter = 0;

    switch (mode) {
        case DisplayMode::SENSITIVITY_CONFIGS:
            if (state.longitudinal_sensitivity == 0) {
                // Manual mode accessed by turning longitudinal sensitivity to zero.
                return print("Manual Mode Lock",
                    String((float)state.lockup * 100.0 / 127.0) + " %");
            }
            return print("Longitudinal: " + String(state.longitudinal_sensitivity),
                         "Lateral:      " + String(state.lateral_sensitivity));
        case DisplayMode::ACCELERATION_CONFIGS:
            return print("Accel begin:  " + String(state.accel_lock_begin),
                         "Ramp width:   " + String(state.accel_ramp_width));
        case DisplayMode::BRAKING_CONFIGS:
            return print("Brake begin:  " + String(state.brake_lock_begin),
                         "Ramp width:   " + String(state.brake_ramp_width));
        case DisplayMode::LATERAL_CONFIGS:
            return print("Latral begin: " + String(state.lateral_lock_begin),
                         "Ramp width:   " + String(state.lateral_ramp_width));
        case DisplayMode::MANUAL_CONTROL:
            return print("Manual Mode:    ",
                         state.manual_mode & 0x01 ? 
                         String(100.0 * (float)state.lockup / 127.0) + "%" : 
                         "             OFF");
        case DisplayMode::STATS:
            switch(subscreen) {
                case 0:
                    return print("Center Diff Lock",
                        String((float)state.lockup * 100.0 / 127.0) + " %");
                case 1:
                    {
                        // horizontal_accel is only used in this case. 
                        // Put it in a block just so the compiler doesn't throw warnings. 
                        float horizontal_accel = sqrt(
                            state.longitudinal_accel * state.longitudinal_accel + 
                            state.lateral_accel * state.lateral_accel);
                        return print("Horizontal Accel",
                            String(horizontal_accel / state.vertical_accel) + " g");
                    }
                case 2:
                    // Angle tracking not implemented
                    subscreen = 0;
                    // return print("Roll Angle:     ",
                    //     String(state.roll_angle * 180.0 / PI) + " degrees");
                case 3:
                    // return print("Pitch Angle:    ",
                    //     String(state.pitch_angle * 180.0 / PI) + " degrees");
                case 4:
                    subscreen = 0;
                    return;
                default:
                    subscreen = 3;
                    return;
            }

        case DisplayMode::INPUTS:
            switch (subscreen) {
                case 0:
                    return print("Longitudinal Acc",
                        String(state.longitudinal_accel / state.vertical_accel) + " g");
                case 1:
                    return print("Lateral Accel:  ",
                        String(state.lateral_accel / state.vertical_accel) + " g");
                case 2:
                    return print("Gravity:        ",
                        String(state.vertical_accel) + " g");
                case 3:
                    // Anglular measurements not currently used
                    subscreen = 0;
                    return;
                    // return print("Yaw Rate:       ",
                    //     String(state.yaw_rate * 180.0 / PI) + " deg/s");
                case 4:
                    // return print("Roll Rate:      ",
                    //     String(state.roll_rate * 180.0 / PI) + " deg/s");
                case 5:
                    // return print("Pitch Rate:     ",
                    //     String(state.pitch_rate * 180.0 / PI) + " deg/s");
                case 6:
                    // Speed measurement not implemented
                    // return print("Speed:          ",
                    //     String(state.longitudinal_speed) + " mph");
                case 7:
                    subscreen = 0;
                    return;
                default:
                    subscreen = 6;
                    return;
            }
    }
}

void Display::delay_UI(unsigned long ms) {
    while(ms > 65) {
        delayMicroseconds(65000);
        ms -= 65;
    }
    delayMicroseconds(ms * 1000);
}

bool Display::is_waiting() {
    if (!_waiting) {
        return false;
    }
    if (micros() > _wait_until) {
        _waiting = false;
        return false;
    }
    return true;
}