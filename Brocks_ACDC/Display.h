#include <Adafruit_RGBLCDShield.h>
#include "math.h"
#include <Arduino.h>
#include <string.h>

enum LcdMode
{
    // CONFIGS = 0,
    STATS = 0,
    INPUTS,
    ERRORS,
    ENUM_END
};

const uint16_t iterationsBetweenPrints = 2;

// LCD display and keypad
Adafruit_RGBLCDShield lcd_keypad;
uint8_t lcdMode;
uint8_t displayMode;
uint16_t printIterationCounter = 0;

void lcd_print(String line_1 = "", String line_2 = "") {
	lcd_keypad.clear();
	lcd_keypad.setCursor(0, 0);
	lcd_keypad.print(line_1.c_str());
	lcd_keypad.setCursor(0, 1);
	lcd_keypad.print(line_2.c_str());
}

void update_display(
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
        ) {
    // print to LCD
    printIterationCounter++;
    if (printIterationCounter < iterationsBetweenPrints) {
        return;
    }
    printIterationCounter = 0;

    lcd_keypad.clear();
    switch (lcdMode) {
        // case LcdMode::CONFIGS:
        //     lcd_print("Longitudinal: " + longitudinal_sensitivity,
        //               "Lateral:      " + lateral_sensitivity);
        case LcdMode::STATS:
            if (displayMode == 0) {
                lcd_print("Center Diff Lock",
                          String((float)lockup * 100.0 / 127.0) + " %");
            } else if (displayMode == 1) {
                float horizontalAccel = sqrt(longitudinalAccel * longitudinalAccel + lateralAccel * lateralAccel);
                lcd_print("Horizontal Accel",
                          String(horizontalAccel / verticalAccel) + " g");
            } else if (displayMode == 2) {
                lcd_print("Roll Angle:     ",
                          String(rollAngle * 180.0 / PI) + " degrees");
            } else if (displayMode == 3) {
                lcd_print("Pitch Angle:    ",
                          String(pitchAngle * 180.0 / PI) + " degrees");
            } else if (displayMode > 3) {
            displayMode = 0;
            } else if (displayMode < 0) {
            displayMode = 3;
            }
            break;

        case LcdMode::INPUTS:
            if (displayMode == 0) {
                lcd_print("Longitudinal Acc",
                          String(longitudinalAccel / verticalAccel) + " g");
            } else if (displayMode == 1) {
                lcd_print("Lateral Accel:  ",
                          String(lateralAccel / verticalAccel) + " g");
            } else if (displayMode == 2) {
                lcd_print("Yaw Rate:       ",
                          String(yawRate * 180.0 / PI) + " deg/s");
            } else if (displayMode == 3) {
                lcd_print("Roll Rate:      ",
                          String(rollRate * 180.0 / PI) + " deg/s");
            } else if (displayMode == 4) {
                lcd_print("Pitch Rate:     ",
                          String(pitchRate * 180.0 / PI) + " deg/s");
            } else if (displayMode == 5) {
                lcd_print("Gravity:        ",
                          String(verticalAccel / 256.0) + " g");
            } else if (displayMode == 6) {
                lcd_print("Speed:          ",
                          String(longitudinalSpeed) + " mph");
            } else if (displayMode > 6) {
                displayMode = 0;
            } else if (displayMode < 0) {
                displayMode = 6;
            };
    }
}