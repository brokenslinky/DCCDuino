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
    if (mode== DisplayMode::CONFIGS) {
        print("Configuration   ");
    } else if (mode == DisplayMode::STATS) {
        print("Status          ");
    } else if (mode == DisplayMode::INPUTS) {
        print("Inputs          ");   
    }
    delay_UI(USER_READ_TIME_MILLIS); // Give the user time to read.
}

void Display::update(
        int   lockup,
        float longitudinalAccel,
        float lateralAccel,
        float verticalAccel,
        float rollRate,
        float pitchRate,
        float yawRate,
        float longitudinalSpeed,
        float rollAngle,
        float pitchAngle,
        uint8_t longitudinal_sensitivity,
        uint8_t lateral_sensitivity
        ) 
{
    // print to LCD
    printIterationCounter++;
    if (printIterationCounter < ITERATIONS_BETWEEN_PRINTS) {
        return;
    }
    printIterationCounter = 0;

    float horizontalAccel = sqrt(longitudinalAccel * longitudinalAccel + lateralAccel * lateralAccel);

    switch (mode) {
        case DisplayMode::CONFIGS:
            if (longitudinal_sensitivity == 0) {
                // Manual mode accessed by turning longitudinal sensitivity to zero.
                return print("Manual Mode Lock",
                    String((float)lockup * 100.0 / 127.0) + " %");
            }
            return print("Longitudinal: " + String(longitudinal_sensitivity),
                         "Lateral:      " + String(lateral_sensitivity));
        case DisplayMode::STATS:
            switch(subscreen) {
                case 0:
                    return print("Center Diff Lock",
                        String((float)lockup * 100.0 / 127.0) + " %");
                case 1:
                    return print("Horizontal Accel",
                        String(horizontalAccel / verticalAccel) + " g");
                case 2:
                    return print("Roll Angle:     ",
                        String(rollAngle * 180.0 / PI) + " degrees");
                case 3:
                    return print("Pitch Angle:    ",
                        String(pitchAngle * 180.0 / PI) + " degrees");
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
                        String(longitudinalAccel / verticalAccel) + " g");
                case 1:
                    return print("Lateral Accel:  ",
                        String(lateralAccel / verticalAccel) + " g");
                case 2:
                    return print("Yaw Rate:       ",
                        String(yawRate * 180.0 / PI) + " deg/s");
                case 3:
                    return print("Roll Rate:      ",
                        String(rollRate * 180.0 / PI) + " deg/s");
                case 4:
                    return print("Pitch Rate:     ",
                        String(pitchRate * 180.0 / PI) + " deg/s");
                case 5:
                    return print("Gravity:        ",
                        String(verticalAccel / 256.0) + " g");
                case 6:
                    return print("Speed:          ",
                        String(longitudinalSpeed) + " mph");
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

    // unsigned long start = micros();
    // _wait_until = start + ms + 1000;
    // _waiting = true;
    // Serial.println(("Start time: " + String(start)).c_str());
    // Serial.println(("End time: " + String(_wait_until)).c_str());
    // if (_wait_until < start) {
    //     while (_wait_until < micros()) {
    //     // Wait for overflow
    //     // This will lock up the entire device, not just the UI.
    //     // Timer overflow should occur infrequently enough that it's not a big deal.
    //     }
    // }
}

bool Display::is_waiting() {
    if (!_waiting) {
        return false;
    }
    if (micros() > _wait_until) {
        _waiting = false;
        return false;
    }
    // Serial.println(("Now: " + String(micros())).c_str());
    // Serial.println(("Wait until: " + String(_wait_until)).c_str());
    return true;
}