#include <Adafruit_RGBLCDShield.h>
#include "math.h"
#include <Arduino.h>
#include <string.h>

#define USER_READ_TIME_MILLIS 1500

enum LcdMode
{
    CONFIGS = 0,
    STATS,
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

void show_mode() {
    if (lcdMode== LcdMode::CONFIGS) {
        lcd_print("Configuration   ");
    } else if (lcdMode == LcdMode::STATS) {
        lcd_print("Status          ");
    } else if (lcdMode == LcdMode::INPUTS) {
        lcd_print("Inputs          ");       
    } else if (lcdMode == LcdMode::ERRORS) {
        lcd_print("Errors          ");
    }
    delay(USER_READ_TIME_MILLIS); // Give the user time to read.
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

    float horizontalAccel = sqrt(longitudinalAccel * longitudinalAccel + lateralAccel * lateralAccel);

    switch (lcdMode) {
        case LcdMode::CONFIGS:
            if (!longitudinal_sensitivity) {
                // Manual mode accessed by turning longitudinal sensitivity to zero.
                return lcd_print("Manual Mode Lock",
                                 String(100.0 * (float)lateral_sensitivity / 15.0) + " %");
            }
            return lcd_print("Longitudinal: " + longitudinal_sensitivity,
                             "Lateral:      " + lateral_sensitivity);
        case LcdMode::STATS:
            switch(displayMode) {
                case 0:
                    return lcd_print("Center Diff Lock",
                                    String((float)lockup * 100.0 / 127.0) + " %");
                case 1:
                    return lcd_print("Horizontal Accel",
                                    String(horizontalAccel / verticalAccel) + " g");
                case 2:
                    return lcd_print("Roll Angle:     ",
                                    String(rollAngle * 180.0 / PI) + " degrees");
                case 3:
                    return lcd_print("Pitch Angle:    ",
                                    String(pitchAngle * 180.0 / PI) + " degrees");
                case 4:
                    displayMode = 0;
                    return;
                default:
                    displayMode = 3;
                    return;
            }

        case LcdMode::INPUTS:
            switch (displayMode) {
                case 0:
                    return lcd_print("Longitudinal Acc",
                                     String(longitudinalAccel / verticalAccel) + " g");
                case 1:
                    return lcd_print("Lateral Accel:  ",
                                     String(lateralAccel / verticalAccel) + " g");
                case 2:
                    return lcd_print("Yaw Rate:       ",
                                     String(yawRate * 180.0 / PI) + " deg/s");
                case 3:
                    return lcd_print("Roll Rate:      ",
                                     String(rollRate * 180.0 / PI) + " deg/s");
                case 4:
                    return lcd_print("Pitch Rate:     ",
                                     String(pitchRate * 180.0 / PI) + " deg/s");
                case 5:
                    return lcd_print("Gravity:        ",
                                     String(verticalAccel / 256.0) + " g");
                case 6:
                    return lcd_print("Speed:          ",
                                     String(longitudinalSpeed) + " mph");
                case 7:
                    displayMode = 0;
                    return;
                default:
                    displayMode = 6;
                    return;
            }
    }
}