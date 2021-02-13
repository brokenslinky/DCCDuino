//set EEPROM addresses for calibration data
#define ZERO_CAL_ADDR        0x00 // 6 floats = 24 bytes
#define SCALE_CAL_ADDR       0x18 // 6 floats = 24 bytes
#define ORIENTATION_CAL_ADDR 0x30 // 4 floats = 16 bytes
#define SPEED_CAL_ADDR       0x40 // 1 float  =  4 bytes
//next available address is 0x44 (68)
//last available addresses is 0x99 (254)

// Need EEPROM to store calibration data
#include <EEPROM.h>

// Get the ability to read/write doubles on EEPROM
void EEPROM_writeFloat(int ee, float value)
{
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
}
double EEPROM_readFloat(int ee)
{
    float value = 0.0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return value;
}


