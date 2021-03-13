#include <Arduino.h>
#include <Wire.h>

// Planning to use 
// ARDUINO UNO WIFI REV2 
// This LCD Keypad - https://www.adafruit.com/product/716

#include <Arduino_LSM6DS3.h>    // Internal IMU on the Uno WiFi Rev2

#include "Storage.h"
#include "Orientation_Matrix.h"
#include "User_Interface.h"

// Pin Assignments
#define SPEEDOMETER_PIN A5
#define POWER_OUT_PIN   45

// Settings
const uint16_t calibrationIterations   = 512;

// Structure for working with orientation corrections.
OrientationMatrix orientation_matrix;

// Working varibales
static int   lockup            = 0;
static float longitudinalAccel = 0.0;
static float lateralAccel      = 0.0;
static float verticalAccel     = 0.0;
static float rollRate          = 0.0;
static float pitchRate         = 0.0;
static float yawRate           = 0.0;

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
float friction;
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
