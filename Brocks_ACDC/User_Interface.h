#include "Display.h"
#include "Storage.h"

#define ANTI_BOUNCE_MILLIS 128 * 4 // * 4 CLK correction

Display display;

bool calibrating             = false;
bool adjusting_sensitivities = false;
bool adjusting_brake         = false;

// In-memory copies of sensitivities
// Declared here so we can adjust them.
// These will be stored as 4 bits each and share 1 byte.
// Maximum value = 15
uint8_t longitudinal_sensitivity = 0;
uint8_t lateral_sensitivity      = 0;
uint8_t brake_lock_begin         = 2; // Normalize to 1 = 0.1g
uint8_t brake_ramp_width         = 5; // Additive to brake_lock_begin. Also normalized to +1 = +.01g 

/** Initialize **/
void init_UI() {
  display.init();
  calibrating              = false;
  adjusting_sensitivities  = false;
  adjusting_brake          = false;
  longitudinal_sensitivity = 0;
  lateral_sensitivity      = 0;
  brake_lock_begin         = 2;
  brake_ramp_width         = 5;
}

/** Adjust the sensitivity configurations based on the provided button input. **/
void adjust(uint8_t button, 
            long EEPROM_ADDR = SENSITIVITIES_ADDR, 
            uint8_t& value_1 = longitudinal_sensitivity,
            uint8_t& value_2 = lateral_sensitivity) {
  switch (button) {
    case BUTTON_UP:
      if (value_1 < 0x0f) {
        value_1++;
      }
      break;
    case BUTTON_DOWN:
      if (value_1 > 0) {
        value_1--;
      }
      break;
    case BUTTON_LEFT:
      if (value_2 > 0) {
        value_2--;
      }
      break;
    case BUTTON_RIGHT:
      if (value_2 < 0x0f) {
        value_2++;
      }
      break;
    case BUTTON_SELECT:
      adjusting_sensitivities = false;
      adjusting_brake = false;
      EEPROM_write_short_pair(EEPROM_ADDR, value_1, value_2);
      display.print("Configuration   ",
                    "saved to EEPROM.");
      display.delay_UI(USER_READ_TIME_MILLIS);
      break;
  }
  display.delay_UI(ANTI_BOUNCE_MILLIS);
}

/** 
 * Check if any of the buttons are pressed.
 * ToDo: This would be better as an interupt.
 **/
void check_user_input() {
  if (display.is_waiting()) {
    return;
  }

  uint8_t button = display.lcd_keypad.readButtons();
  if (!button) {
    return;
  }

  if (adjusting_sensitivities) {
    return adjust(button, SENSITIVITIES_ADDR, longitudinal_sensitivity, lateral_sensitivity);
  } else if (adjusting_brake) {
    return adjust(button, BRAKE_THRESHOLDS_ADDR, brake_lock_begin, brake_ramp_width);
  }

  // Special behavior for directional buttons on the config screens
  if (display.mode == DisplayMode::CONFIGS && button != BUTTON_SELECT) {
    adjusting_sensitivities = true;
    return adjust(button, SENSITIVITIES_ADDR, longitudinal_sensitivity, lateral_sensitivity);
  }else if (display.mode == DisplayMode::CONFIGS_2 && button != BUTTON_SELECT) {
    adjusting_brake = true;
    return adjust(button, BRAKE_THRESHOLDS_ADDR, brake_lock_begin, brake_ramp_width);
  }

  switch (button) {
    case BUTTON_RIGHT:
      display.subscreen++;
      return display.delay_UI(ANTI_BOUNCE_MILLIS);
    case BUTTON_LEFT:
      display.subscreen--;
      return display.delay_UI(ANTI_BOUNCE_MILLIS);
    case BUTTON_DOWN:
    case BUTTON_SELECT:
      display.subscreen = 0;
      if (display.mode == DisplayMode::ENUM_END - 1) {
        display.mode = 0;
      } else {
        display.mode++;
      }
      return display.show_mode();
    case BUTTON_UP:
      display.subscreen = 0;
      if (display.mode == 0) {
        display.mode = DisplayMode::ENUM_END - 1;
      } else {
        display.mode--;
      }
      return display.show_mode();
  }
}
