#include <Arduino.h>
#include <Wire.h>

// Planning to use 
// ARDUINO UNO WIFI REV2 

#include <Arduino_LSM6DS3.h> // Internal IMU on the Uno WiFi Rev2

#include "Storage.h"
#include "Orientation_Matrix.h"
#include "User_Interface.h"

// Pin Assignments
#define SPEEDOMETER_PIN A3
#define DIFF_LOCK_PIN   9

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
float accelScale[3];
float accelOffset[3];
float gyroScale[3];
float gyroOffset[3];

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
