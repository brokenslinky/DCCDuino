/**
 * Used to write a default calibration to the EEPROM.
**/

#include <Arduino.h>
#include <Wire.h>

#include <math.h>

#include "Storage.h" 
// Since Arduino wants a flat file structure, 
// I have a symlink of Storage.h in my local directory Initial_Setup_Tools.
// This symlink is untracked by the git repository.
// You can create a symlink or just copy the file from the parent directory.

float zero_vector[]   = {0.0, 0.0, 0.0}; // Default for offsets
float one_vector[]    = {1.0, 1.0, 1.0}; // Default for scaling

float default_angle   = radians(60.0);
float sixty_degrees[] = {0.0, -sin(default_angle), cos(default_angle)};
// x = left, y = down-back, z = away from screen

void print_stored_vector(int EEPROM_ADDR, String descrition = "") {
    float vector[3];
    EEPROM_read_vector(EEPROM_ADDR, vector);
    for (int i = 0; i < 3; i++) {
        Serial.println((descrition + "["+String(i)+"]: " + String(vector[i])).c_str());
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    EEPROM_write_vector(ACCEL_ZERO_ADDR,      zero_vector);
    EEPROM_write_vector(GYRO_ZERO_ADDR,       zero_vector);
    EEPROM_write_vector(ACCEL_SCALE_ADDR,     one_vector);
    EEPROM_write_vector(GYRO_SCALE_ADDR,      one_vector);
    EEPROM_write_vector(ORIENTATION_CAL_ADDR, sixty_degrees);
    EEPROM_write_short_pair(SENSITIVITIES_ADDR, 0, 0);
    // This Represents no offsets or rescaling, 60 degree mounting pitch, no roll

    // Verify the values were stored and can be read back.
    Serial.println("Wrote default calibration to EEPROM:");
    print_stored_vector(ACCEL_ZERO_ADDR,      "Accel Zero");
    print_stored_vector(GYRO_ZERO_ADDR,       "Gyro Zero");
    print_stored_vector(ACCEL_SCALE_ADDR,     "Accel Scale");
    print_stored_vector(GYRO_SCALE_ADDR,      "Gyro Scale");
    print_stored_vector(ORIENTATION_CAL_ADDR, "Orientation");
    uint8_t longitudinal_sensitivity;
    uint8_t lateral_sensitivity;
    EEPROM_read_short_pair(SENSITIVITIES_ADDR, longitudinal_sensitivity, lateral_sensitivity);
    Serial.println(("Longitudinal Sensitivity: " + String(longitudinal_sensitivity)).c_str());
    Serial.println(("Lateral Sensitivity: " + String(lateral_sensitivity)).c_str());
}

void loop() {
    // Nothing
}