#include <Arduino.h>
#include <Wire.h>
#include "Storage.h"

// Planning to use 
// ARDUINO UNO WIFI REV2 
// This LCD Keypad - https://www.adafruit.com/product/716
// Some hokey rotary switches I picked up years ago

#include <Arduino_LSM6DS3.h> // Internal IMU on the Uno WiFi Rev2
#include <Adafruit_RGBLCDShield.h>
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

// Calibration variables
float speedCal;
float gyroOffset[3];
float zxgr, zygr, xg, yg, zg, rg; // to store calibration data in SRAM
const int calibrationIterations = 512;
float accelZero[3];
float accelScale[3];

// Slip angle and speed;
float slip;
float speedoPeriod;
float longitudinalSpeed;
float rollAngle;
float pitchAngle;
float rampRate;

// LCD display and keypad
Adafruit_RGBLCDShield lcd_keyad;

uint8_t lcdMode;
uint8_t displayMode; // Not set enums. Varies based on LcdMode.
const uint16_t iterationsBetweenPrints = 2;
uint16_t printIterationCounter = 0;

// All of this crap is for a speed calibration.
float previousSpeed = 0;
uint32_t previousCalTime = 0;
float previousAccel = 0;
float savt;
float svt2;
float speedCorrection = 1.0;
boolean toss = true;
uint16_t numberSpeedData = 0;

float slipError;
int turnEnd;
int turnStart;
boolean turning;
int iterationsDuringTurn;
int yawSum;
int previousIterationsDuringTurn;
int previousYawSum;


// Declared after setup


unsigned long lastTick = 0;
unsigned long interval = 1;
unsigned long previousIteration = 0;
unsigned long iterationTime = 0;
unsigned long time = 0;
boolean speedo = false;
boolean calibrationMode = false;

float rotationAdder = 0;
float yawAdder = 0;
int counter = 0;
int i = 0;

float unadjusted_accel[3];
float unadjusted_rotation[3];