#ifndef STORAGE_H
#define STORAGE_H
//set EEPROM addresses for calibration data

#define ACCEL_ZERO_ADDR         0x00 // (0)  3 floats = 12 bytes
#define GYRO_ZERO_ADDR          0x0c // (12) 3 floats = 12 bytes
#define ACCEL_SCALE_ADDR        0x18 // (24) 3 floats = 12 bytes
#define GYRO_SCALE_ADDR         0x24 // (36) 3 floats = 12 bytes
#define ORIENTATION_CAL_ADDR    0x30 // (48) 3 floats = 12 bytes
#define SENSITIVITIES_ADDR      0x3c // (60) 2 4-bits =  1 byte
#define BRAKE_THRESHOLDS_ADDR   0x3d // (61) 2 4-bits =  1 byte
#define ACCEL_THRESHOLDS_ADDR   0x3e // (62) 2 4-bits =  1 byte
#define LATERAL_THRESHOLDS_ADDR 0x3f // (63) 2 4-bits =  1 byte
#define MANUAL_CONTROLS_ADDR    0x40 // (64) 2 4-bits =  1 byte

//next available address is 0x41 (65)
//last available addresses is 0xff (255)

#include <EEPROM.h>

/** Write a floating point number to EEPROM **/
void EEPROM_write_float(int EEPROM_addr, float value) {
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++) {
        EEPROM.write(EEPROM_addr++, *p++);
    }
}

/** Read a floating point number from EEPROM **/
float EEPROM_read_float(int EEPROM_addr) {
    float value = 0.0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++) {
        *p++ = EEPROM.read(EEPROM_addr++);
    }
    return value;
}

/** Write a floating point vector to EEPROM **/
void EEPROM_write_vector(int EEPROM_addr, float* array, int dimensions = 3) {
    for (int i = 0; i < dimensions; i++) {
        EEPROM_write_float(EEPROM_addr + i * sizeof(float), array[i]);
    }
}

/** Read a 3-dimensional float from the EEPROM_addr and writes to the provided pointer **/
void EEPROM_read_vector(int EEPROM_addr, float* returned_vector) {
    byte* p = (byte*)(void*)returned_vector;
    for (int i = 0; i < sizeof(float[3]); i++) {
        *p++ = EEPROM.read(EEPROM_addr++);
    }
}

/** Save a pair of 4-bit objects to EEPROM **/
void EEPROM_write_short_pair(int EEPROM_addr, uint8_t first, uint8_t second) {
    // These values should be <16. Package them together in 1 byte.
    uint8_t tmp = (first & 0x0f) << 4 | (second & 0x0f);
    EEPROM.write(EEPROM_addr, tmp);
}

/** Read a pair of 4-bit objects from EEPROM and write to the provided in-memory references **/
void EEPROM_read_short_pair(int EEPROM_addr, uint8_t &first, uint8_t &second) {
    uint8_t tmp = EEPROM.read(EEPROM_addr);
    first = (tmp & 0xf0) >> 4; // first 4 bits in the address
    second = tmp & 0x0f; // last 4 bits in the address
}

#endif /* STORAGE_H */