#include "Brocks_ACDC.h"

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  lcd_keyad.begin(16, 2);
  lcd_keyad.print("ACDC-duino by   ");
  lcd_keyad.setCursor(0,1);
  lcd_keyad.print("   Brock Palmer ");

  lcdMode = 0;
  displayMode = 0;
  printIterationCounter = 0;

  pinMode(SPEEDOMETER_PIN, INPUT);
  pinMode(CALIBRATION_PIN, INPUT);
  pinMode(SLICKNESS_PIN,   INPUT);
  pinMode(RAMP_PIN,        INPUT);
  pinMode(LED_GROUND_PIN,  INPUT);

  pinMode(POWER_OUT_PIN,   OUTPUT);
  pinMode(RED_PIN,         OUTPUT);
  pinMode(BLUE_PIN,        OUTPUT);
  pinMode(GREEN_PIN,       OUTPUT);
  
  // Initialize working variables
  slip                         = 0;
  speedoPeriod                 = 200.0; // 0.1 coresponds to ~100 mph
  rollAngle                    = 0;
  pitchAngle                   = 0;
  slipError                    = 0;
  turnStart                    = 0;
  turnEnd                      = 1;
  iterationsDuringTurn         = 0;
  previousIterationsDuringTurn = 0;
  yawSum                       = 0;
  previousYawSum               = 0;
  
  // Prepare LED
  digitalWrite(LED_GROUND_PIN, LOW);
  // Display orange light for initialization
  analogWrite(RED_PIN, 255);
  analogWrite(GREEN_PIN, 125);
  analogWrite(BLUE_PIN, 0);

  float orientationCal[4];

  // Default calibration values... delete these after calibration
  speedCal = 1024; // <a_y/v>/<yaw>
  orientationCal[0] = 1/2; // x/g
  orientationCal[1] = 0; // y/g
  orientationCal[2] = sqrt(3)/2; // z/g
  orientationCal[4] = 1/2; // r/g
  for (int i = 0; i < 3; i++) 
  {
    accelZero[i] = 0;
    accelScale[i] = 0.1;
  }
  // This estimates no yaw offset, 30 degree pitch, no roll, perfect sero balance and scale

  // Pull calibration data from EEPROM
  lcd_keyad.clear();
  lcd_keyad.setCursor(0,0);
  lcd_keyad.print("Getting cal data");
  lcd_keyad.setCursor(0,1);
  lcd_keyad.print("from EEPROM...  ");
  speedCal = EEPROM_readFloat(SPEED_CAL_ADDR);
  for (int i = 0; i < 3; i++) 
  {
    accelZero[i] = EEPROM_readFloat(ZERO_CAL_ADDR + 4 * i);
    gyroOffset[i] = EEPROM_readFloat(ZERO_CAL_ADDR + (3 + i) * 4);
    accelScale[i] = EEPROM_readFloat(SCALE_CAL_ADDR + 4 * i);
  }
  for (int i = 0; i<4; i++)
  {
    orientationCal[i] = EEPROM_readFloat(ORIENTATION_CAL_ADDR + 4 * i);
  }
  // Assign calibration data to RAM
  zxgr = orientationCal[2] * orientationCal[0] / orientationCal[3];
  zygr = orientationCal[2] * orientationCal[1] / orientationCal[3];
  xg = orientationCal[0];
  yg = orientationCal[1];
  zg = orientationCal[2];
  rg = orientationCal[3];

  /*
  // zero the gyros
  delay(500);
  for (int j = 0; j < 3; j ++)
  {
    gyroOffset[j] = 0;
  }
  for (int i = 0; i < calibrationIterations; i++)
  {
    IMU.readGyroscope(unadjusted_rotation[0], unadjusted_rotation[1], unadjusted_rotation[2])
    for (int j = 0; j<3; j++)
    {
      gyroOffset[j] += unadjusted_rotation[j];
    }
  }
  for (int j = 0; j<3; j++)
  {
    gyroOffset[j] /= calibrationIterations;
  }
  */

  // Display blue light to indicate readiness
  analogWrite(RED_PIN, 0);
  analogWrite(GREEN_PIN, 0);
  analogWrite(BLUE_PIN, 255);
  lcd_keyad.clear();
  lcd_keyad.setCursor(0,0);
  lcd_keyad.print("Ready to race.  ");
}

void loop()
{
  checkSpeedo();
  lcd_keyad.setCursor(0,0);
  
  IMU.readAcceleration(unadjusted_accel[0], unadjusted_accel[1], unadjusted_accel[2]);
  IMU.readGyroscope(unadjusted_rotation[0], unadjusted_rotation[1], unadjusted_rotation[2]);
  
  /*double accelerationX = unadjusted_accel[0] * 0.038305; // in m/s^2 //0.00390625; // in Gs
  double accelerationY = unadjusted_accel[1] * 0.038305;
  double yawRate = unadjusted_rotation[5] * 1.0793385355081877697;*/    // in 10,000 rad/s
  float longitudinalAccelRaw = zxgr * (unadjusted_accel[0] - accelZero[0]) * accelScale[0] - zygr * (unadjusted_accel[1] - accelZero[1]) * accelScale[1] - xg * (unadjusted_accel[2] - accelZero[2]) * accelScale[2];
  float lateralAccelRaw = zygr * (unadjusted_accel[0] - accelZero[0]) * accelScale[0] + zxgr * (unadjusted_accel[1] - accelZero[1]) * accelScale[1] + yg * (unadjusted_accel[2] - accelZero[2]) * accelScale[2];
  float verticalAccelRaw = zg * (unadjusted_accel[2] - accelZero[2]) * accelScale[2] + rg * sqrt((unadjusted_accel[0] - accelZero[0]) * accelScale[0] * (unadjusted_accel[0] - accelZero[0]) * accelScale[0] + (unadjusted_accel[1] - accelZero[1]) * accelScale[1] * (unadjusted_accel[1] - accelZero[1]) * accelScale[1]);
  float rollRateRaw = zxgr * (unadjusted_rotation[0] - gyroOffset[0]) - zygr * (unadjusted_rotation[1] - gyroOffset[1]) - xg * (unadjusted_rotation[2] - gyroOffset[2]);
  float pitchRateRaw = zygr * (unadjusted_rotation[0] - gyroOffset[0]) + zxgr * (unadjusted_rotation[1] - gyroOffset[1]) + yg * (unadjusted_rotation[2] - gyroOffset[2]);
  float yawRateRaw = zg * (unadjusted_rotation[2] - gyroOffset[2]) + rg * sqrt((unadjusted_rotation[0] - gyroOffset[0]) * (unadjusted_rotation[0] - gyroOffset[0]) + (unadjusted_rotation[1] - gyroOffset[1]) * (unadjusted_rotation[1] - gyroOffset[1]));  
  
  //convert gyros to SI units
  rollRateRaw *= 0.00106529694631448; // rad/s
  pitchRateRaw *= 0.00106529694631448;
  yawRateRaw *= 0.00106529694631448;
  
  // correct IMU inputs for pitch/roll
  /*
  float rollRate = rollRateRaw * cos(pitchAngle) + yawRateRaw * sin(pitchAngle);  
  float pitchRate = pitchRateRaw * cos(rollAngle) - yawRateRaw * sin(rollAngle);
  float yawRate = yawRateRaw * cos(rollAngle) * cos(pitchAngle) + pitchRateRaw * sin(rollAngle) - rollRateRaw * sin(pitchAngle);
  float longitudinalAccel = longitudinalAccelRaw * cos(pitchAngle) + verticalAccelRaw * sin(pitchAngle);
  float lateralAccel = lateralAccelRaw * cos(rollAngle) - verticalAccelRaw * sin(rollAngle);
  float verticalAccel = verticalAccelRaw * cos(pitchAngle) * cos(rollAngle) - longitudinalAccelRaw * sin(pitchAngle) + lateralAccelRaw * sin(rollAngle);
  */ // Unlock above and delete below if the gyros are upgraded 
  
  float rollRate = rollRateRaw;  
  float pitchRate = pitchRateRaw;
  float yawRate = yawRateRaw;
  float longitudinalAccel = longitudinalAccelRaw;
  float lateralAccel = lateralAccelRaw;
  float verticalAccel = verticalAccelRaw;
  rollAngle += rollRate * iterationTime / 1000000;
  pitchAngle += pitchRate * iterationTime / 1000000;
  // these angles are cleared at the same time as slip angle
  
  if (lcd_keyad.readButtons() == BUTTON_SELECT)
  {
    // while car is stationary, calibrate orientation and zero gyros
    if (calibrationMode == true) 
    {
      calibrationMode = false;
      
      //speedCal = abs(rotationAdder / yawAdder);
      
      // display green light for "on"
      analogWrite(RED_PIN, 0);
      analogWrite(BLUE_PIN, 0);
      analogWrite(GREEN_PIN, 255);
      lcd_keyad.clear();
      lcd_keyad.print("Writing to ROM..");
      lcd_keyad.setCursor(0,1);
      lcd_keyad.print("Do not poweroff.");
      lcd_keyad.setCursor(0,0);
      
      //EEPROM_writeFloat(SPEED_CAL_ADDR, speedCal); Restore this when speedo works
      
      lcd_keyad.clear();
      lcd_keyad.print("Cal successful.");
      lcd_keyad.setCursor(0,1);
      lcd_keyad.print("Ready to go.");
      lcd_keyad.setCursor(0,0);
    }
    else
    {
      calibrationMode = true;
      float x = 0;
      float y = 0;
      float z = 0;
      float r = 0;
      float g = 0;
      for (int j = 0; j<3; j++)
      {
        gyroOffset[j] = 0;
      }
      float tmp[3];

      lcd_keyad.clear();
      lcd_keyad.print("  Measuring     ");
      lcd_keyad.setCursor(0,1);
      lcd_keyad.print("  Orientation   ");

      for (int i = 0; i < calibrationIterations; i++)
      {
        // Display orange light for wait
        analogWrite(RED_PIN, 255);
        analogWrite(GREEN_PIN, 150);
        analogWrite(BLUE_PIN, 0);

        
        // Determine direction of gravity
        IMU.readAcceleration(unadjusted_accel[0], unadjusted_accel[1], unadjusted_accel[2]);
        for ( int j = 0; j < 3; j++)
        {
          tmp[j] = (unadjusted_accel[j] - accelZero[j]) * accelScale[j];
        }
        x += tmp[0];
        y += tmp[1];
        z += tmp[2];
        r += sqrt(tmp[0] * tmp[0] + tmp[1] * tmp[1]);
        g += sqrt(tmp[0] * tmp[0] + tmp[1] * tmp[1] + tmp[2] * tmp[2]);
        // may as well update gyro offsets too
        IMU.readGyroscope(unadjusted_rotation[0], unadjusted_rotation[1], unadjusted_rotation[2]);
        for (int j = 0; j < 3; j++)
        {
          gyroOffset[j] += unadjusted_rotation[j];
        }

        delay(2);
      }
      // save orientation to ROM and RAM
      EEPROM_writeFloat(ORIENTATION_CAL_ADDR, x/g);
      EEPROM_writeFloat(ORIENTATION_CAL_ADDR + 4, y/g);
      EEPROM_writeFloat(ORIENTATION_CAL_ADDR + 8, z/g);
      EEPROM_writeFloat(ORIENTATION_CAL_ADDR + 12, r/g);
      zxgr = z * x / (g * r);
      zygr = z * y / (g * r);
      xg = x / g;
      yg = y / g;
      zg = z / g;
      rg = r / g;
      
      // prepare rotation and yaw variables for speed calibration
      rotationAdder = 0;
      yawAdder = 0;
      
      // convert gyroOffset from sum to average
      for (int j = 0; j < 3; j++)
      {
        gyroOffset[j] /= calibrationIterations;
        EEPROM_writeFloat(ZERO_CAL_ADDR + 4 * (3 + j), gyroOffset[j]);
      }
      
      // prepare to enter speed calibration mode
      calibrationMode = true;
      // Display purple light for speed calibration
      analogWrite(RED_PIN, 255);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, 255);
      lcd_keyad.clear();
      lcd_keyad.setCursor(0,0);
      lcd_keyad.print("Drive in a circle.");
      lcd_keyad.setCursor(0,1);
      lcd_keyad.print("DO NOT SLIDE!");
      lcd_keyad.setCursor(0,0);  
      
      
      // Erase this when speedo works
      calibrationMode = false;
      //speedCal = abs(rotationAdder / yawAdder);
      // display green light for "on"
      analogWrite(RED_PIN, 0);
      analogWrite(BLUE_PIN, 0);
      analogWrite(GREEN_PIN, 255);
      lcd_keyad.clear();
      lcd_keyad.print("Writing to ROM..");
      lcd_keyad.setCursor(0,1);
      lcd_keyad.print("Do not poweroff.");
      lcd_keyad.setCursor(0,0);
      
      lcd_keyad.clear();
      lcd_keyad.print("Cal successful.");
      lcd_keyad.setCursor(0,1);
      lcd_keyad.print("Ready to go.");
      lcd_keyad.setCursor(0,0);
      
    }
  }
  
  /* Resotre this when speedo works
  if (calibrationMode == true)
  {
    if ((rotationAdder + lateralAccel * speedoPeriod != rotationAdder) && (yawAdder + yawRate != yawAdder) && (speedoPeriod < 200)) 
    {
      rotationAdder += lateralAccel * speedoPeriod;
      yawAdder += yawRate;
    }
    if (rotationAdder > 4194304) 
    {
      // display yellow to red light to warn user of float storage
      analogWrite(RED_PIN, 255);
      if (rotationAdder < 8388608) analogWrite(GREEN_PIN, 255 - (rotationAdder - 4194304) / 16450);
      else analogWrite(GREEN_PIN, 255);
      analogWrite(BLUE_PIN, 0);
      lcd_keyad.clear();
      lcd_keyad.print("WARNING: float limitation");
      lcd_keyad.setCursor(0,1);
      lcd_keyad.print("End calibration.");
      lcd_keyad.setCursor(0,0);
    }
  }
  */

  
  time = micros();
  if (time - previousIteration > 0) iterationTime = time - previousIteration;
  previousIteration = time;
  
  /*
  //Use Runge-Kutta to keep track of slip angle
  float k1 = dphidt(yawRate, lateralAccel, slip, longitudinalAccel, speedCal, speedoPeriod);
  float k2 = dphidt(yawRate, lateralAccel, slip + 0.5 * iterationTime * k1 / 1000000.0, longitudinalAccel, speedCal, speedoPeriod);
  float k3 = dphidt(yawRate, lateralAccel, slip + 0.5 * iterationTime * k2 / 1000000.0, longitudinalAccel, speedCal, speedoPeriod);
  float k4 = dphidt(yawRate, lateralAccel, slip + iterationTime * k3 / 1000000.0, longitudinalAccel, speedCal, speedoPeriod);
  
  slip += (k1 + 0.5 * k2 + 0.5 * k3 + k4) * iterationTime / 6000000.0;
  if (slip > 2 * PI) slip -= 2 * PI;
  */
  
  // reset the slip angle if lateral acceleration is small
  if (abs(lateralAccel) < 25.6) // ~0.1g 
  {
    if (turning == true)
    {
      turnEnd = millis();
      turning = false;
      previousYawSum = yawSum;
      previousIterationsDuringTurn = iterationsDuringTurn;
      yawSum = 0;
      iterationsDuringTurn = 0;
      slipError = slip;
    }
    slip = 0;
    rollAngle = 0;
  }
  else 
  {
    if (turning == false)
    {
      turnStart = millis();
      turning = true;
    }
    yawSum += yawRate;
    iterationsDuringTurn++;
  }
    
  if (abs(longitudinalAccel) < 25.6) //.1 g
  {
    pitchAngle = 0;
  }
  
  // read the aggression and slip dials
  float friction = 2.4 - 0.2 * getRotaryKey(analogRead(SLICKNESS_PIN));
  //float desiredSlip = 0.0872664626 * getRotaryKey(analogRead(RAMP_PIN)); // 5 degrees per position
  rampRate = 1.0 / ( 3.0 - 0.25 * getRotaryKey(analogRead(RAMP_PIN)));
  
  float horizontalAccel = sqrt(longitudinalAccel * longitudinalAccel + lateralAccel * lateralAccel);
  float tmp = 127 * (rampRate * 0.04559 * abs(yawRate) * longitudinalSpeed / ( abs(lateralAccel) * cos(slip) * cos(slip) + longitudinalAccel * cos(slip) * sin(slip)) + 1.625 * (longitudinalAccel * cos(slip) - abs(lateralAccel) * sin(slip)) / ( friction * verticalAccel) - 1);
  
  // determine lockup amount (127 = 50% duty cycle MAX)
  int lockup = 0;
  //if (desiredSlip < 0.05) lockup = 127 * slickness / 12; // full lock if slip dial set to zero... don't try the math or NaN will happen
  if (rampRate == 0) lockup = 127 * getRotaryKey(analogRead(SLICKNESS_PIN)) / 11; // manual mode if rampRate set to zero
  else lockup = tmp;
  if (lockup > 127) lockup = 127; // don't exceed 50% duty cycle
  if (lockup < 0) lockup = 0; // no negative PWM values
  
  // send PWM signal to power shield
  analogWrite(POWER_OUT_PIN, lockup);
  // also display on LED - green to red
  analogWrite(RED_PIN, 2 * lockup);
  analogWrite(GREEN_PIN, 254 - 2 * lockup);
  analogWrite(BLUE_PIN, 0);
  
  // collect speed calibration datum
  time = micros();
  if (abs(longitudinalAccel) > 0.25 && time > previousCalTime && numberSpeedData < 2166)
  {
    if (toss == true)
    {
      toss = false;
    }
    else
    {
      float dvdt = 1000.0 * (longitudinalSpeed - previousSpeed) / (time - previousCalTime); //mph/s
      float a = 21.94 * (longitudinalAccel - previousAccel) / 2; //mph/s
      previousSpeed = longitudinalSpeed;
      previousAccel = longitudinalAccel;
      savt += a * dvdt;
      svt2 += dvdt * dvdt;
      numberSpeedData++;
    }
  }
  else 
    toss = true;
  speedCorrection = savt/svt2;
  
  // print to LCD
  printIterationCounter++;
  if (printIterationCounter > iterationsBetweenPrints)
  {
    printIterationCounter = 0;
    lcd_keyad.clear();
    if(lcdMode == LcdMode::STATS)
    {
      if (displayMode == 0)
      {
          lcd_keyad.print("Center diff lock:");
          lcd_keyad.setCursor(0,1);
          lcd_keyad.print((float)lockup * 100.0 / 127.0);
          lcd_keyad.print(" %");
      }
      if (displayMode == 1)
      {
        // lcd_keyad.print("Slip Angle:");
        lcd_keyad.print("Hoizontal Accel:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(horizontalAccel / verticalAccel);
        lcd_keyad.print(" g");
        //lcd_keyad.print(slip * 180.0 / PI);
        //lcd_keyad.print(" degrees");
      }
      if (displayMode == 2)
      {
        lcd_keyad.print("Roll Angle:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(rollAngle * 180.0 / PI);
        lcd_keyad.print(" degrees");
      }
      if (displayMode == 3)
      {
        lcd_keyad.print("Pitch Angle:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(pitchAngle * 180.0 / PI);
        lcd_keyad.print(" degrees");
      }
      if (displayMode > 3) displayMode = 0;
      if (displayMode < 0) displayMode = 3;
    }
    if (lcdMode == LcdMode::INPUTS)
    {
      if (displayMode == 0)
      {
        lcd_keyad.print("Longitudinal Acc:");
        lcd_keyad.setCursor(0,1);
        // lcd_keyad.print(longitudinalAccel / 256.0);
        lcd_keyad.print(longitudinalAccel / verticalAccel);
        lcd_keyad.print(" g");
      }
      if (displayMode == 1)
      {
        lcd_keyad.print("Lateral Accel:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(lateralAccel / verticalAccel);
        lcd_keyad.print(" g");
      }
      if (displayMode == 2)
      {
        lcd_keyad.print("Yaw Rate:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(yawRate * 180.0 / PI);
        lcd_keyad.print(" deg/s");
      }
      if (displayMode == 3)
      {
        lcd_keyad.print("Roll Rate:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(rollRate * 180.0 / PI);
        lcd_keyad.print(" deg/s");
      }
      if (displayMode == 4)
      {
        lcd_keyad.print("Pitch Rate:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(pitchRate * 180.0 / PI);
        lcd_keyad.print(" deg/s");
      }
      if (displayMode == 5)
      {
        lcd_keyad.print("Gravity:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(verticalAccel / 256.0);
        lcd_keyad.print(" g");
      }
      if (displayMode == 6)
      {
        lcd_keyad.print("Speed:");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(longitudinalSpeed);
        lcd_keyad.print(" mph");
      }   
      if (displayMode > 6) displayMode = 0;
      if (displayMode < 0) displayMode = 6;
    }


    /*
    if(lcdMode == LcdMode::ERRORS)
    {
      if (displayMode == 0)
      {
        lcd_keyad.print("Yaw rate error: ");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(0.1 * slipError * (float) previousIterationsDuringTurn / ((float) previousYawSum * ((float) turnEnd - (float) turnStart)));
        lcd_keyad.print(" %");
      }
      if (displayMode == 1)
      {
        lcd_keyad.print("Slip error: ");
        lcd_keyad.setCursor(0,1);
        lcd_keyad.print(slipError * 180.0 / PI);
        lcd_keyad.print(" deg");
      }
      if (displayMode > 1) displayMode = 0;
      if (displayMode < 0) displayMode = 1;
    }
    */
  }
  
  // keypad buttons
  if (lcd_keyad.readButtons() == BUTTON_RIGHT) 
  {
    displayMode++;
    delay(200);
  }
  if (lcd_keyad.readButtons() == BUTTON_LEFT) 
  {
    displayMode--;
    delay(200);
  }
  if (lcd_keyad.readButtons() == BUTTON_DOWN)
  {
    displayMode = 0;
    if (lcdMode == LcdMode::ENUM_END - 1) lcdMode = 0;
    else lcdMode++;
    if (lcdMode == LcdMode::STATS)
    {
      lcd_keyad.clear();
      lcd_keyad.print("Status");
      delay(2000);
    }
    if (lcdMode == LcdMode::INPUTS)
    {
      lcd_keyad.clear();
      lcd_keyad.print("Inputs");
      delay(2000);
    }
    if (lcdMode == LcdMode::ERRORS)
    {
      lcd_keyad.clear();
      lcd_keyad.print("Errors");
      delay(2000);
    }
  }
  if (lcd_keyad.readButtons() == BUTTON_UP)
  {
    displayMode = 0;
    if (lcdMode == 0) lcdMode = LcdMode::ENUM_END - 1;
    else lcdMode--;
    if (lcdMode == LcdMode::STATS)
    {
      lcd_keyad.clear();
      lcd_keyad.print("Status");
      delay(2000);
    }
    if (lcdMode == LcdMode::INPUTS)
    {
      lcd_keyad.clear();
      lcd_keyad.print("Inputs");
      delay(2000);
    }
    if (lcdMode == LcdMode::ERRORS)
    {
      lcd_keyad.clear();
      lcd_keyad.print("Errors");
      delay(2000);
    }
  }

  //delay(2); // avoid exceeding PWM frequency
}

void checkSpeedo()
{
  if (speedoPeriod > 175.0 && calibrationMode == false) //coresponds to ~5mph
  {
    // display pretty teal light for readiness
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 255);
    analogWrite(BLUE_PIN, 150);
    if (lcdMode == 0 && printIterationCounter == 0 && displayMode == 0 && rampRate != 0)
    {
      //lcd_keyad.clear();
      lcd_keyad.print("We are parked...");
      lcd_keyad.setCursor(0,1);
      lcd_keyad.print("Ready & waiting...");
      lcd_keyad.setCursor(0,0);
      delay(32);
    }
  }
  if (analogRead(SPEEDOMETER_PIN) > 64) // ~.3125V
  {
    if (speedo == false)
    {      
      speedo = true;
      time = micros();
      if (time - lastTick > 0) 
        speedoPeriod = (time - lastTick) / 1000.0; // milliseconds
      lastTick = time;
      longitudinalSpeed = /*speedCorrection **/ 877.193 / speedoPeriod; //+0.125; //approximate mph
    }
  }
  if (analogRead(SPEEDOMETER_PIN) < 32) // ~0.15625V
  {
    speedo = false;
  }
}

/* Fix speed correction factor before using
float dphidt(float yawRate, float lateralAccel, float slipAngle, float longitudinalAccel, float speedScale, float period)
{
  float dslipdt = abs(yawRate) - (abs(lateralAccel) * cos(slipAngle) + longitudinalAccel * sin(slip)) * period / speedScale;
  return dslipdt;
}
*/
