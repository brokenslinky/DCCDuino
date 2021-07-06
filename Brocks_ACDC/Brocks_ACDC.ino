#include "Brocks_ACDC.h"

void setup() {
  Serial.begin(9600);
  Wire.begin();
  IMU.begin();
  init_UI();

  display.print("ACDCduino by    ",
                "   Brock Palmer ");
  // Orange light
  display.set_rgb(255, 125, 0);

  // I want to see the splash screen because I am vane.
  display.delay_UI(USER_READ_TIME_MILLIS);

  pinMode(SPEEDOMETER_PIN,  INPUT);
  pinMode(DIFF_LOCK_PIN,    OUTPUT);

  // Change PWM frequency of DIFF_LOCK_PIN
  // Pin 9 should be on TCA0-WO0
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | 0x1; 
  // Default frequency is 976 Hz
  // 1/8 of this should be 122 Hz
  // I think 0x1 is already used. This is a divisor of 4, so I want a divisor of 32.
  // See https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173B.pdf 

  // Initialize working variables
  speedoPeriod        = 100.0; // 0.1 coresponds to ~100 mph. 100.0 coresponds to ~0.1 mph
  state.roll_angle    = 0.0;
  state.slip          = 0.0;
  state.pitch_angle   = 0.0;

  float orientationCal[3];

  // Default calibration values
  // orientationCal[0] = sqrt(3)/2; // x/g
  // orientationCal[1] = 0;         // y/g
  // orientationCal[2] = 1/2;       // z/g
  // for (int i = 0; i < 3; i++) {
  //   accelOffset[i]  = 0;
  //   accelScale[i] = 0.1;
  // }
  // This estimates no yaw offset, 60 degree pitch, no roll, perfect zero balance and scale

  // Pull calibration data and configuration from EEPROM
  display.print("Getting cal data", 
                "from EEPROM...  ");
  EEPROM_read_vector(ACCEL_ZERO_ADDR,             accelOffset);
  EEPROM_read_vector(GYRO_ZERO_ADDR,              gyroOffset);
  EEPROM_read_vector(ACCEL_SCALE_ADDR,            accelScale);
  EEPROM_read_vector(GYRO_SCALE_ADDR,             gyroScale);
  EEPROM_read_vector(ORIENTATION_CAL_ADDR,        orientationCal);
  EEPROM_read_short_pair(SENSITIVITIES_ADDR,      state.longitudinal_sensitivity, state.lateral_sensitivity);
  EEPROM_read_short_pair(BRAKE_THRESHOLDS_ADDR,   state.brake_lock_begin,         state.brake_ramp_width);
  EEPROM_read_short_pair(ACCEL_THRESHOLDS_ADDR,   state.accel_lock_begin,         state.accel_ramp_width);
  EEPROM_read_short_pair(LATERAL_THRESHOLDS_ADDR, state.lateral_lock_begin,       state.lateral_ramp_width);
  EEPROM_read_short_pair(MANUAL_CONTROLS_ADDR,    state.manual_lock_amount,       state.manual_mode);

  // Create calibration matrix from orientation data
  orientation_matrix.update(orientationCal);

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
  float x = (unadjusted_accel[0] - accelOffset[0]) * accelScale[0];
  float y = (unadjusted_accel[1] - accelOffset[1]) * accelScale[1];
  float z = (unadjusted_accel[2] - accelOffset[2]) * accelScale[2];
  float x_rot = (unadjusted_rotation[0] - gyroOffset[0]) * gyroScale[0];
  float y_rot = (unadjusted_rotation[1] - gyroOffset[1]) * gyroScale[1];
  float z_rot = (unadjusted_rotation[2] - gyroOffset[2]) * gyroScale[2];

  // Convert from sensor coordinates to vehicle coordinates.
  state.longitudinal_accel = orientation_matrix.longitudinal(x, y, z);
  state.lateral_accel      = orientation_matrix.lateral     (x, y, z);
  state.vertical_accel     = orientation_matrix.vertical    (x, y, z);
  state.roll_rate          = orientation_matrix.longitudinal(x_rot, y_rot, z_rot);
  state.pitch_rate         = orientation_matrix.lateral     (x_rot, y_rot, z_rot);
  state.yaw_rate           = orientation_matrix.vertical    (x_rot, y_rot, z_rot);
  
  //convert gyros to SI units
  state.roll_rate  *= 0.01745329251; // rad/s
  state.pitch_rate *= 0.01745329251; // pi / 180
  state.yaw_rate   *= 0.01745329251;

  time = micros();
  if (time - previousIteration > 0) {
    iterationTime = time - previousIteration;
  }
  previousIteration = time;
  
  // Track orientation changes
  state.roll_angle  += state.roll_rate  * iterationTime / 1000000;
  state.pitch_angle += state.pitch_rate * iterationTime / 1000000;
  // Slip angle is more complicated since the reference frame is free to rotate in the yaw direction
  // these angles are zeroed when lateral acceleration is small
    
  // Zero the orientation any time the car is inertial.
  if (fabs(state.longitudinal_accel) < 0.05 && fabs(state.lateral_accel) < 0.05) {
    state.pitch_angle = 0.0;
    state.roll_angle  = 0.0;
    state.slip        = 0.0;
  }

  // determine lockup amount (127 = 50% duty cycle MAX)
  state.lockup = LockingAlgorithms::calculate_lock(state);
  
  // send PWM signal to power shield
  analogWrite(DIFF_LOCK_PIN, state.lockup);

  // LED changes from cyan to red and becomes brighter as the diff locks.
  // display.set_rgb(2 * lockup, 64 - lockup / 2.0, 64 - lockup / 2.0);
  // 0 cyan -> 25 green -> 50 yellow -> 75 red 
  display.set_rgb(2 * state.lockup, 223 - state.lockup, 159 - state.lockup);

  display.update(state);
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
      state.longitudinal_speed = /*speedCorrection **/ 877.193 / speedoPeriod; //+0.125; //approximate mph
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
    for ( int j = 0; j < 3; j++) {
      tmp[j] = (unadjusted_accel[j] - accelOffset[j]) * accelScale[j];
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
  EEPROM_write_vector(GYRO_ZERO_ADDR, gyroOffset);

  display.print("  Calibration   ",
                "   Complete     ");
  display.set_rgb(0, 255, 0); // green light
}
