// This is the LCD Keypad I used - https://www.adafruit.com/product/716

#include <Adafruit_RGBLCDShield.h> // https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library

#include "math.h"
#include <Arduino.h>
#include <string.h>

#define USER_READ_TIME_MILLIS   1000 * 4 // * 4 CLK correction
#define ITERATIONS_BETWEEN_PRINTS  8

// Pins for RGB light
#define RED_PIN   3
#define GREEN_PIN 5
#define BLUE_PIN  6

enum DisplayMode
{
    CONFIGS = 0,
    STATS,
    INPUTS,
    ENUM_END
};

struct Display
{
    // LCD display and keypad
    Adafruit_RGBLCDShield lcd_keypad;

    uint8_t               mode;
    uint8_t               subscreen;
    uint16_t              printIterationCounter;

    Display() {};
    ~Display() {};

    /** Initialize the display **/
    void init();

    /** Print to the display. Limit 2 lines, 16 characters per line. **/
    void print(String line_1 = "", String line_2 = "");

    /** Set the RGB LED **/
    void set_rgb(uint8_t red, uint8_t green, uint8_t blue);

    /** Display the name of the current display mode **/
    void show_mode();

    /** Update the display based on the provided state **/
    void update(
            int   lockup,
            float longitudinalAccel,
            float lateralAccel,
            float verticalAccel,
            float rollRate,
            float pitchRate,
            float yawRate,
            float longitudinalSpeed,
            float rollAngle,
            float pitchAngle,
            uint8_t longitudinal_sensitivity,
            uint8_t lateral_sensitivity
            );

    /** Prevent any user inputs or screen refreshes for the provided time. **/
    void delay_UI(unsigned long ms);

    /** Should the UI be waiting on a previously requested delay? **/
    bool is_waiting();

    private:
    bool _waiting = false;
    unsigned long _wait_until;
};