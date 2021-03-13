#include "Display.h"
#include "Storage.h"

#define ANTI_BOUNCE_MILLIS 100

bool calibrating = false;
bool adjusting   = false;

// In-memory copies of sensitivities
// Declared here so the can adjust them.
// These will be stored as 4 bits each and share 1 byte.
// Maximum value = 15
uint8_t longitudinal_sensitivity = 0;
uint8_t lateral_sensitivity      = 0;

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
      lcd_print("Configuration   ",
                "saved to EEPROM.");
      return delay(USER_READ_TIME_MILLIS);
  }
  delay(ANTI_BOUNCE_MILLIS);
}

void read_buttons() {
  uint8_t button = lcd_keypad.readButtons();
  if (!button) {
    return;
  }

  if (adjusting) {
    return adjust(button);
  }

  // Special behavior for directional buttons on the config screen
  if (lcdMode == LcdMode::CONFIGS && button != BUTTON_SELECT) {
    adjusting = true;
    return adjust(button);
  }

  switch (button) {
    case BUTTON_RIGHT:
      displayMode++;
      return delay(ANTI_BOUNCE_MILLIS);
    case BUTTON_LEFT:
      displayMode--;
      return delay(ANTI_BOUNCE_MILLIS);
    case BUTTON_DOWN:
    case BUTTON_SELECT:
      displayMode = 0;
      if (lcdMode == LcdMode::ENUM_END - 1) {
        lcdMode = 0;
      } else {
        lcdMode++;
      }
      return show_mode();
    case BUTTON_UP:
      displayMode = 0;
      if (lcdMode == 0) {
        lcdMode = LcdMode::ENUM_END - 1;
      } else {
        lcdMode--;
      }
      return show_mode();
  }
}
