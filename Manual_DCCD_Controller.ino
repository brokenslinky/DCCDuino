/* This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.*/

// Call libraries    
#include <Arduino.h>
#include <Wire.h>

// Assign pins
#define MOSFET 9
#define adjustmentKnob A1
#define ledGround 3
#define red 4
#define green 5
#define blue 6

// You may need to re-key the rotary dial here. 
// use Serial.println(analogRead,rotaryKnob); to determine keys
#define NUM_KEYS 12
int rotaryKeyVal[NUM_KEYS - 1] = {790,841,875,905,920,934,945,955,965,975,985};
int getRotaryKey(unsigned int input)
{
    int k;
    for (k = 0; k < NUM_KEYS; k++)
    {
      if (input < rotaryKeyVal[k])
     {
       return k;
     }
   }
       if (k >= NUM_KEYS)k = 12;      
       return k;
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(MOSFET, OUTPUT);
  pinMode(adjustmentKnob, INPUT);
  pinMode(ledGround, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  digitalWrite(ledGround, LOW);
}

int lock = 0;

void loop()
{
  lock = 127 * getRotaryKey(analogRead(adjustmentKnob)) / 11;
  //Serial.println(analogRead(adjustmentKnob)); // use this to determine keys
  if (lock > 127)
    {
      lock = 127;
    }
  analogWrite(MOSFET, lock);
  ledSubroutine(lock);
}

int ledSubroutine(int x)
{
  if(x < 43)
  {
    analogWrite(blue, x*2);
    analogWrite(green, 0);
    digitalWrite(red, LOW);
    return 0;
  }
  if(x < 86)
  {
    analogWrite(blue, 172-x*2);
    analogWrite(green, x*4-172);
    digitalWrite(red, LOW);
    return 0;
  }
  analogWrite(blue, 0);
  analogWrite(green, x*2);
  digitalWrite(red, 0);
}

