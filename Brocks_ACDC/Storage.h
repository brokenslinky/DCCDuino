//set EEPROM addresses for calibration data

#define ZERO_CAL_ADDR        0x00 // 6 floats = 24 bytes
#define SCALE_CAL_ADDR       0x18 // 6 floats = 24 bytes
#define ORIENTATION_CAL_ADDR 0x30 // 4 floats = 16 bytes

//next available address is 0x40 (48)
//last available addresses is 0x99 (254)

#include <EEPROM.h>

// Get the ability to read/write doubles on EEPROM
void EEPROM_write_float(int ee, float value) {
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++) {
        EEPROM.write(ee++, *p++);
    }
}

float EEPROM_read_float(int ee) {
    float value = 0.0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++) {
        *p++ = EEPROM.read(ee++);
    }
    return value;
}

void EEPROM_write_vector(int addr, float value[3]) {
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(float[3]); i++) {
        EEPROM.write(addr++, *p++);
    }
}

/**
 * Reads a 3-dimensional float from the EEPROM_addr and writes to the provided pointer
 */
void EEPROM_read_vector(int EEPROM_addr, float* returned_vector) {
    byte* p = (byte*)(void*)returned_vector;
    for (int i = 0; i < sizeof(float[3]); i++) {
        *p++ = EEPROM.read(EEPROM_addr++);
    }
}