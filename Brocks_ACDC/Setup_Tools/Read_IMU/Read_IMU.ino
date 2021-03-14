/**
 * Used to read the accelerometer and gyrometer on a bench to determine offsets and scaling factors.
**/

#include <Arduino.h>
#include <Wire.h>

#include <Arduino_LSM6DS3.h> // Internal IMU on the Uno WiFi Rev2

#include "Display.h"
// Since Arduino wants a flat file structure, 
// I have symlinks in my local directory to files from the parent directory.
// The symlinks are untracked by the git repository.
// You can create your own symlinks or just copy the necessary files from the parent directory.

#define NUMBER_OF_SAMPLES         128
#define TIME_BETWEEN_SAMPLES_MILLIS 8

Display display;

// Working Variables
float acceleration_sum[3];
float gyrometer_sum[3];
float avg_acceleration[3];
float avg_gyrometer[3];
int   reading_iterator;

void measure_averages() {
    for (int i = 0; i < 3; i++) {
        acceleration_sum[i] = 0.0;
        gyrometer_sum[i]    = 0.0;
    }

    for (int i = 0; i < NUMBER_OF_SAMPLES; i++) {
        float accelerometer[3];
        float gyrometer[3];
        IMU.readAcceleration(accelerometer[0], accelerometer[1], accelerometer[2]);
        IMU.readGyroscope(gyrometer[0], gyrometer[1], gyrometer[2]);
        for (int i = 0; i < 3; i++) {
            acceleration_sum[i] += accelerometer[i];
            gyrometer_sum[i]    += gyrometer[i];
        }
        delay(TIME_BETWEEN_SAMPLES_MILLIS);
    }

    for (int i = 0; i < 3; i++) {
        avg_acceleration[i] = acceleration_sum[i] / NUMBER_OF_SAMPLES;
        avg_gyrometer[i]   = gyrometer_sum[i] / NUMBER_OF_SAMPLES;
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    display.init();

    measure_averages();

    reading_iterator = 0;
}

void loop() {
    if (reading_iterator >= 6) {
        reading_iterator = 0;
    }

    if (reading_iterator < 3) {
        float average_reading = avg_acceleration[reading_iterator];
        display.print("Acceleration[" + String(reading_iterator) + "]:",
                      String(average_reading));
        Serial.println("Acceleration[" + String(reading_iterator) + "]:" + String(average_reading));
    } else {
        float average_reading = avg_gyrometer[reading_iterator - 3];
        display.print("Rotation[" + String(reading_iterator - 3) + "]:    ",
                      String(average_reading));
        Serial.println("Rotation[" + String(reading_iterator - 3) + "]:" + String(average_reading));
    }

    reading_iterator++;
}