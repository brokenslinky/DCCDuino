#include <Adafruit_RGBLCDShield.h>
#include "math.h"
#include <Arduino.h>

enum LcdMode
{
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

void lcd_print(const char* line_1 = "", const char* line_2 = "") {
	lcd_keypad.clear();
	lcd_keypad.setCursor(0, 0);
	lcd_keypad.print(line_1);
	lcd_keypad.setCursor(0, 1);
	lcd_keypad.print(line_2);
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
        float rampRate
        ) {
    // print to LCD
    printIterationCounter++;
    if (printIterationCounter < iterationsBetweenPrints) {
        return;
    }
    printIterationCounter = 0;
    lcd_keypad.clear();
    switch (lcdMode) {
    case LcdMode::STATS:
        if (displayMode == 0) {
        lcd_keypad.print("Center diff lock:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print((float)lockup * 100.0 / 127.0);
        lcd_keypad.print(" %");
        } else if (displayMode == 1) {
        float horizontalAccel = sqrt(longitudinalAccel * longitudinalAccel + lateralAccel * lateralAccel);
        lcd_keypad.print("Hoizontal Accel:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(horizontalAccel / verticalAccel);
        lcd_keypad.print(" g");
        } else if (displayMode == 2) {
        lcd_keypad.print("Roll Angle:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(rollAngle * 180.0 / PI);
        lcd_keypad.print(" degrees");
        } else if (displayMode == 3) {
        lcd_keypad.print("Pitch Angle:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(pitchAngle * 180.0 / PI);
        lcd_keypad.print(" degrees");
        } else if (displayMode > 3) {
        displayMode = 0;
        } else if (displayMode < 0) {
        displayMode = 3;
        }
        break;

    case LcdMode::INPUTS:
        if (displayMode == 0) {
        lcd_keypad.print("Longitudinal Acc:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(longitudinalAccel / verticalAccel);
        lcd_keypad.print(" g");
        } else if (displayMode == 1) {
        lcd_keypad.print("Lateral Accel:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(lateralAccel / verticalAccel);
        lcd_keypad.print(" g");
        } else if (displayMode == 2) {
        lcd_keypad.print("Yaw Rate:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(yawRate * 180.0 / PI);
        lcd_keypad.print(" deg/s");
        } else if (displayMode == 3) {
        lcd_keypad.print("Roll Rate:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(rollRate * 180.0 / PI);
        lcd_keypad.print(" deg/s");
        } else if (displayMode == 4) {
        lcd_keypad.print("Pitch Rate:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(pitchRate * 180.0 / PI);
        lcd_keypad.print(" deg/s");
        } else if (displayMode == 5) {
        lcd_keypad.print("Gravity:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(verticalAccel / 256.0);
        lcd_keypad.print(" g");
        } else if (displayMode == 6) {
        lcd_keypad.print("Speed:");
        lcd_keypad.setCursor(0,1);
        lcd_keypad.print(longitudinalSpeed);
        lcd_keypad.print(" mph");
        } else if (displayMode > 6) {
        displayMode = 0;
        } else if (displayMode < 0) {
        displayMode = 6;
        };
    }
}