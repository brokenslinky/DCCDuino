// Libraries    
#include <Arduino.h>
#include <Wire.h>

// Assign pins
#define MOSFET 9
#define ADJUSTMENT_KNOB A1
#define LED_GROUND 3
#define RED 4
#define GREEN 5
#define BLUE 6

// You may need to re-key the rotary dial here. 
// use Serial.println(analogRead(ADJUSTMENT_KNOB)); to determine appropriate keys
#define NUM_KEYS 12
int rotaryKeyVal[NUM_KEYS - 1] = {790,841,875,905,920,934,945,955,965,975,985}; // These values should be between the possible outputs from the rotary switch at its 12 positions.
int getRotaryKey(unsigned int input)
{
    int k;
    for (k = 0; k < NUM_KEYS; k++) {
      if (input < rotaryKeyVal[k]) {
        return k;
      }
    }
    if (k >= NUM_KEYS) {
      k = NUM_KEYS;
    } 
    return k;
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(MOSFET, OUTPUT);
  pinMode(ADJUSTMENT_KNOB, INPUT);
  pinMode(LED_GROUND, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(LED_GROUND, LOW);
}

void loop()
{
  int lock = 127 * getRotaryKey(analogRead(ADJUSTMENT_KNOB)) / (NUM_KEYS - 1);
  //Serial.println(analogRead(ADJUSTMENT_KNOB)); // use this to determine appropriate keys
  if (lock > 127) {
      lock = 127; // Never exceed 50% duty cycle. The diff will not lock more and it will just create heat.
    }
  analogWrite(MOSFET, lock);
  ledSubroutine(lock);
}

void ledSubroutine(int lock)
{
  // Example of a light which you can use to help troubleshoot. Transitions from green to red as the lock increases.
  analogWrite(RED, 2 * lock);
  analogWrite(GREEN, 255 - 2 * lock);
  analogWrite(BLUE, 0);
}
