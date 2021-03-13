#include "Display.h"

bool calibrating = false;

void read_buttons() {
  uint8_t button = lcd_keypad.readButtons();
  switch (button) {
    case 0x00:
      break;
    case BUTTON_RIGHT:
      displayMode++;
      delay(200);
    case BUTTON_LEFT:
      displayMode--;
      delay(200);
    case BUTTON_DOWN:
      displayMode = 0;
      if (lcdMode == LcdMode::ENUM_END - 1) {
        lcdMode = 0;
      } else {
        lcdMode++;
      }
      if        (lcdMode == LcdMode::STATS) {
        lcd_keypad.clear();
        lcd_keypad.print("Status");
        delay(2000);
      } else if (lcdMode == LcdMode::INPUTS) {
        lcd_keypad.clear();
        lcd_keypad.print("Inputs");
        delay(2000);
      } else if (lcdMode == LcdMode::ERRORS) {
        lcd_keypad.clear();
        lcd_keypad.print("Errors");
        delay(2000);
      }

    case BUTTON_UP:
      displayMode = 0;
      if (lcdMode == 0) {
        lcdMode = LcdMode::ENUM_END - 1;
      } else {
        lcdMode--;
      }
      if        (lcdMode == LcdMode::STATS) {
        lcd_keypad.clear();
        lcd_keypad.print("Status");
        delay(2000);
      } else if (lcdMode == LcdMode::INPUTS) {
        lcd_keypad.clear();
        lcd_keypad.print("Inputs");
        delay(2000);
      } else if (lcdMode == LcdMode::ERRORS) {
        lcd_keypad.clear();
        lcd_keypad.print("Errors");
        delay(2000);
      }
  }
}
