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
float sixty_degrees[] = {sin(default_angle), 0.0, cos(default_angle)};

void setup() {
    Serial.begin(9600);
    Wire.begin();

    EEPROM_write_vector(ACCEL_ZERO_ADDR,      zero_vector);
    EEPROM_write_vector(GYRO_ZERO_ADDR,       zero_vector);
    EEPROM_write_vector(ACCEL_SCALE_ADDR,     one_vector);
    EEPROM_write_vector(GYRO_SCALE_ADDR,      one_vector);
    EEPROM_write_vector(ORIENTATION_CAL_ADDR, sixty_degrees);
    EEPROM_write_vector(SENSITIVITIES_ADDR,   0);
    // This Represents no offsets or rescaling, 60 degree mounting pitch, no roll
}

void loop() {
    // Nothing
}