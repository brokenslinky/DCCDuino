#ifndef STORAGE_H
#define STORAGE_H
//set EEPROM addresses for calibration data

#define ZERO_CAL_ADDR        0x00 // 6 floats = 24 bytes
#define SCALE_CAL_ADDR       0x18 // 6 floats = 24 bytes
#define ORIENTATION_CAL_ADDR 0x30 // 4 floats = 16 bytes
#define SENSITIVITIES_ADDR   0x40 // 2 4-bits =  1 byte

//next available address is 0x41 (49)
//last available addresses is 0x99 (254)

#include <EEPROM.h>

/// Get the ability to read/write doubles on EEPROM
void EEPROM_write_float(int EEPROM_addr, float value) {
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++) {
        EEPROM.write(EEPROM_addr++, *p++);
    }
}

float EEPROM_read_float(int EEPROM_addr) {
    float value = 0.0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++) {
        *p++ = EEPROM.read(EEPROM_addr++);
    }
    return value;
}

void EEPROM_write_vector(int EEPROM_addr, float value[3]) {
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(float[3]); i++) {
        EEPROM.write(EEPROM_addr++, *p++);
    }
}

/// Reads a 3-dimensional float from the EEPROM_addr and writes to the provided pointer
void EEPROM_read_vector(int EEPROM_addr, float* returned_vector) {
    byte* p = (byte*)(void*)returned_vector;
    for (int i = 0; i < sizeof(float[3]); i++) {
        *p++ = EEPROM.read(EEPROM_addr++);
    }
}

/// Save the in-memory values of sensitivities to ROM
void EEPROM_write_short_pair(int EEPROM_addr, uint8_t first, uint8_t second) {
    // These values should be <16. Package them together in 1 byte.
    uint8_t tmp = (first & 0x0f) << 4 | (second & 0x0f);
    EEPROM.write(SENSITIVITIES_ADDR, tmp);
}

/// Populate in-memory values of sensitivities from ROM
void EEPROM_read_short_pair(int EEPROM_addr, uint8_t &first, uint8_t &second) {
    uint8_t tmp = EEPROM.read(EEPROM_addr);
    first = (tmp & 0xf0) >> 4; // first 4 bits in the address
    second = tmp & 0x0f; // last 4 bits in the address
}

#endif /* STORAGE_H */