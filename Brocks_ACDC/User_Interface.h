#include "Display.h"
#include "Storage.h"

#define ANTI_BOUNCE_MILLIS 100

Display display;

bool calibrating = false;
bool adjusting   = false;

// In-memory copies of sensitivities
// Declared here so the can adjust them.
// These will be stored as 4 bits each and share 1 byte.
// Maximum value = 15
uint8_t longitudinal_sensitivity = 0;
uint8_t lateral_sensitivity      = 0;

/** Initialize **/
void init_UI() {
  display.init();
  calibrating              = false;
  adjusting                = false;
  longitudinal_sensitivity = 0;
  lateral_sensitivity      = 0;
}

/** Adjust the sensitivity configurations based on the provided button input. **/
void adjust(uint8_t button) {
  switch (button) {
    case BUTTON_UP:
      if (longitudinal_sensitivity < 0x0f) {
        longitudinal_sensitivity++;
      }
      break;
    case BUTTON_DOWN:
      if (longitudinal_sensitivity > 0) {
        longitudinal_sensitivity--;
      }
      break;
    case BUTTON_LEFT:
      if (lateral_sensitivity > 0) {
        lateral_sensitivity--;
      }
      break;
    case BUTTON_RIGHT:
      if (lateral_sensitivity < 0x0f) {
        lateral_sensitivity++;
      }
      break;
    case BUTTON_SELECT:
      adjusting = false;
      EEPROM_write_short_pair(SENSITIVITIES_ADDR, longitudinal_sensitivity, lateral_sensitivity);
      display.print("Configuration   ",
                    "saved to EEPROM.");
      return delay(USER_READ_TIME_MILLIS);
  }
  delay(ANTI_BOUNCE_MILLIS);
}

/** 
 * Check if any of the buttons are pressed.
 * ToDo: This would be better as an interupt.
 **/
void check_user_input() {
  uint8_t button = display.lcd_keypad.readButtons();
  if (!button) {
    return;
  }

  if (adjusting) {
    return adjust(button);
  }

  // Special behavior for directional buttons on the config screen
  if (display.mode == DisplayMode::CONFIGS && button != BUTTON_SELECT) {
    adjusting = true;
    return adjust(button);
  }

  switch (button) {
    case BUTTON_RIGHT:
      display.subscreen++;
      return delay(ANTI_BOUNCE_MILLIS);
    case BUTTON_LEFT:
      display.subscreen--;
      return delay(ANTI_BOUNCE_MILLIS);
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
