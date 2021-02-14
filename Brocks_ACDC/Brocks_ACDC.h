#include <Arduino.h>
#include <Wire.h>

// Planning to use 
// ARDUINO UNO WIFI REV2 
// This LCD Keypad - https://www.adafruit.com/product/716
// Some hokey rotary switches I picked up years ago

#include <Arduino_LSM6DS3.h> // Internal IMU on the Uno WiFi Rev2
#include <Adafruit_RGBLCDShield.h>

#include "Storage.h"
#include "Orientation_Matrix.h"
#include "Rotary_Keys.h"     // Maps analog input to a switch position

// Pin Assignments
#define SPEEDOMETER_PIN A5
#define CALIBRATION_PIN A3
#define POWER_OUT_PIN   45
#define SLICKNESS_PIN   A1
#define RAMP_PIN        A2
#define BLUE_PIN        3
#define GREEN_PIN       47
#define RED_PIN         49
#define LED_GROUND_PIN  51

enum LcdMode
{
	STATS = 0,
	INPUTS,
	ERRORS,
	ENUM_END
};

// Settings
const uint16_t calibrationIterations   = 512;
const uint16_t iterationsBetweenPrints = 2;


// Set the RGB LED
void led_light(uint8_t red, uint8_t green, uint8_t blue) {
  analogWrite(RED_PIN,   red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN,  blue);
}

// LCD display and keypad
Adafruit_RGBLCDShield lcd_keyad;
uint8_t lcdMode;
uint8_t displayMode;
uint16_t printIterationCounter;

// Structure for working with orientation corrections.
OrientationMatrix orientation_matrix;

void lcd_print(const char* line_1 = "", const char* line_2 = "") {
	lcd_keyad.clear();
	lcd_keyad.setCursor(0, 0);
	lcd_keyad.print(line_1);
	lcd_keyad.setCursor(0, 1);
	lcd_keyad.print(line_2);
}

// Sensor readings
float unadjusted_accel[3];
float unadjusted_rotation[3];

// Calibration variables
float gyroOffset[3];
float accelZero[3];
float accelScale[3];

// Slip angle and speed;
float slip;
float speedoPeriod;
float longitudinalSpeed;
float rollAngle;
float pitchAngle;
float rampRate;

// All of this crap is for a speed calibration.
float previousSpeed      = 0.0;
uint32_t previousCalTime = 0;
float previousAccel      = 0.0;
float savt;
float svt2;
float speedCorrection    = 1.0;
boolean toss = true;
uint16_t numberSpeedData = 0;

// Variables for dynamic corrections
unsigned long lastTick = 0;
unsigned long interval = 1;
unsigned long previousIteration = 0;
unsigned long iterationTime = 0;
unsigned long time = 0;

bool speedo_triggered = false;
bool calibrationMode  = false;
