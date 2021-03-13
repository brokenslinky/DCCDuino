#include "Brocks_ACDC.h"

void setup() {
  Serial.begin(9600);
  Wire.begin();
  init_UI();

  display.print("ACDCduino by    ",
                "   Brock Palmer ");
  // Orange light
  display.set_rgb(255, 125, 0);

  pinMode(SPEEDOMETER_PIN, INPUT);
  pinMode(POWER_OUT_PIN,   OUTPUT);

  // Change PWM frequency of POWER_OUT_PIN
  // Pin 9 should be on TCA0-WO0
  // TODO: Confirm this works
  // TCA0.SINGLE.CTRLA = (TCA0.SINGLE.CTRLA & 0xF1) | (0x4 << 1);
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc; // "| 0x1 might be needed to enable."
  // Default frequency is 976 Hz
  // 1/8 of this should be 122 Hz
  // I think 0x1 is already used. This is a divisor of 4, so I want a divisor of 32.
  // See https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173B.pdf 

  // Initialize working variables
  speedoPeriod = 100.0; // 0.1 coresponds to ~100 mph. 100.0 coresponds to ~0.1 mph
  rollAngle    = 0.0;
  slip         = 0.0;
  pitchAngle   = 0.0;

  float orientationCal[3];

  // Default calibration values
  // orientationCal[0] = sqrt(3)/2; // x/g
  // orientationCal[1] = 0;         // y/g
  // orientationCal[2] = 1/2;       // z/g
  // for (int i = 0; i < 3; i++) {
  //   accelZero[i]  = 0;
  //   accelScale[i] = 0.1;
  // }
  // This estimates no yaw offset, 60 degree pitch, no roll, perfect zero balance and scale

  // Pull calibration data and configuration from EEPROM
  display.print("Getting cal data", 
                "from EEPROM...  ");
  EEPROM_read_vector(ZERO_CAL_ADDR,        accelZero);
  EEPROM_read_vector(ZERO_CAL_ADDR + 3*4,  gyroOffset);
  EEPROM_read_vector(SCALE_CAL_ADDR,       accelScale);
  EEPROM_read_vector(ORIENTATION_CAL_ADDR, orientationCal);
  EEPROM_read_short_pair(SENSITIVITIES_ADDR, longitudinal_sensitivity, lateral_sensitivity);

  // Create calibration matrix from orientation data
  orientation_matrix.update(orientationCal);

  // I want to see the splash screen because I am vane.
  delay(USER_READ_TIME_MILLIS);

  // Display blue light to indicate readiness
  display.set_rgb(0, 0, 255);
  display.print("Ready to race.  ");
}

void loop() {
  // Check the speedometer signal every iteration to make sure no pulses are missed.
  checkSpeedo();
  check_user_input();
  
  // Refresh data from Inertial Measurement Unit.
  IMU.readAcceleration(unadjusted_accel[0], unadjusted_accel[1], unadjusted_accel[2]);
  IMU.readGyroscope(unadjusted_rotation[0], unadjusted_rotation[1], unadjusted_rotation[2]);

  // Zero and scale the sensor data.
  float x = (unadjusted_accel[0] - accelZero[0]) * accelScale[0];
  float y = (unadjusted_accel[1] - accelZero[1]) * accelScale[1];
  float z = (unadjusted_accel[2] - accelZero[2]) * accelScale[2];
  float x_rot = (unadjusted_rotation[0] - accelZero[0]) * accelScale[0];
  float y_rot = (unadjusted_rotation[1] - accelZero[1]) * accelScale[1];
  float z_rot = (unadjusted_rotation[2] - accelZero[2]) * accelScale[2];

  // Convert from sensor coordinates to vehicle coordinates.
  longitudinalAccel = orientation_matrix.longitudinal(x, y, z);
  lateralAccel      = orientation_matrix.lateral     (x, y, z);
  verticalAccel     = orientation_matrix.vertical    (x, y, z);
  rollRate          = orientation_matrix.longitudinal(x_rot, y_rot, z_rot);
  pitchRate         = orientation_matrix.lateral     (x_rot, y_rot, z_rot);
  yawRate           = orientation_matrix.vertical    (x_rot, y_rot, z_rot);
  
  //convert gyros to SI units
  rollRate  *= 0.01745329251; // rad/s
  pitchRate *= 0.01745329251; // pi / 180
  yawRate   *= 0.01745329251;

  time = micros();
  if (time - previousIteration > 0) {
    iterationTime = time - previousIteration;
  }
  previousIteration = time;
  
  // Track orientation changes
  rollAngle  += rollRate  * iterationTime / 1000000;
  pitchAngle += pitchRate * iterationTime / 1000000;
  // Slip angle is more complicated since the reference frame is free to rotate in the yaw direction
  // these angles are zeroed when lateral acceleration is small
    
  // Zero the orientation any time the car is inertial.
  if (abs(longitudinalAccel) < 0.1 && abs(lateralAccel) < 0.1) {
    pitchAngle = 0.0;
    rollAngle  = 0.0;
    slip       = 0.0;
  }
  
  // determine lockup amount (127 = 50% duty cycle MAX)
  if (!longitudinal_sensitivity) {
    // Manual mode is accessed by reducing the longitudinal sensitivity to zero.
    lockup = 127.0 * lateral_sensitivity / 15; // manual mode if rampRate set to zero
  } else {
    friction = 1.5 - 0.1 * (float)longitudinal_sensitivity;
    rampRate = 1.0 / ( 3.01 - 0.2 * (float)lateral_sensitivity);
    float lock_from_acceleration = 1.625 * (longitudinalAccel * cos(slip) - abs(lateralAccel) * sin(slip)) / ( friction * verticalAccel) - 1;
    float lock_from_yaw_rate     = rampRate * 0.04559 * abs(yawRate) * longitudinalSpeed / 
                                   (abs(lateralAccel) * cos(slip) * cos(slip) + longitudinalAccel * cos(slip) * sin(slip));
    lockup = 127.0 * (lock_from_acceleration + lock_from_yaw_rate);
  }

  if (lockup > 127.0) {
    lockup = 127.0; // don't exceed 50% duty cycle
  }
  if (lockup < 0.0) {
    lockup = 0.0; // no negative PWM values
  }
  
  // send PWM signal to power shield
  analogWrite(POWER_OUT_PIN, lockup);

  // LED changes from cyan to red and becomes brighter as the diff locks.
  display.set_rgb(2 * lockup, 64 - lockup / 2.0, 64 - lockup / 2.0);

  display.update(
        lockup,
        longitudinalAccel,
        lateralAccel,
        verticalAccel,
        rollRate,
        pitchRate,
        yawRate,
        longitudinalSpeed,
        rollAngle,
        pitchAngle,
        friction,
        rampRate
        );
}

/** 
 * Need to check for a pulse from the speedometer signal frequently enough to catch every pulse.
 * ToDo: Use bluetooth to read speed from the car's ECU.
 *       If that's not possible, I think this can be set up as an interupt.
 **/
void checkSpeedo() {
  if (analogRead(SPEEDOMETER_PIN) > 64 /* ~0.3125V */) {
    if (!speedo_triggered) {      
      speedo_triggered = true;
      time = micros();
      if (time - lastTick > 0) 
        speedoPeriod = (time - lastTick) / 1000.0; // milliseconds
      lastTick = time;
      longitudinalSpeed = /*speedCorrection **/ 877.193 / speedoPeriod; //+0.125; //approximate mph
    }
  }
  if (analogRead(SPEEDOMETER_PIN) < 32 /* ~0.15625V */) {
    speedo_triggered = false;
  }
}

/// while car is stationary, calibrate orientation and zero gyros
void perform_calibration() {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  float g = 0.0;
  for (int j = 0; j<3; j++) {
    gyroOffset[j] = 0;
  }
  float tmp[3];

  display.print("  Measuring     ", 
                "  Orientation   ");

  for (int i = 0; i < calibrationIterations; i++) {
    // Display orange light for wait
    display.set_rgb(255, 150, 0);
    
    // Determine direction of gravity
    IMU.readAcceleration(unadjusted_accel[0], unadjusted_accel[1], unadjusted_accel[2]);
    for ( int j = 0; j < 3; j++)
    {
      tmp[j] = (unadjusted_accel[j] - accelZero[j]) * accelScale[j];
    }
    x += tmp[0];
    y += tmp[1];
    z += tmp[2];
    g += sqrt(tmp[0] * tmp[0] + tmp[1] * tmp[1] + tmp[2] * tmp[2]);

    // Zero the gyro.
    IMU.readGyroscope(unadjusted_rotation[0], unadjusted_rotation[1], unadjusted_rotation[2]);
    for (int j = 0; j < 3; j++) {
      gyroOffset[j] += unadjusted_rotation[j];
    }

    delay(2); // Give the IMU time to refresh.
  }

  // save orientation to ROM and RAM
  {
    float orientation[3] = { x / g, y / g, z / g };
    EEPROM_write_vector(ORIENTATION_CAL_ADDR, orientation);
    orientation_matrix.update(orientation);
  }
  
  // convert gyroOffset from sum to average and write to EEPROM
  for (int j = 0; j < 3; j++) {
    gyroOffset[j] /= calibrationIterations;
  }
  EEPROM_write_vector(ZERO_CAL_ADDR + 4*3, gyroOffset);

  display.print("  Calibration   ",
                "   Complete     ");
  display.set_rgb(0, 255, 0); // green light
}
