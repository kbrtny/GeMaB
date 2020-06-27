#include "MotorDriver.h"


#if defined(REDBOARD_TURBO)
  TurboPWM pwm;
#endif

MotorDriver::MotorDriver()
{
  _M1nSLEEP = 2;
  _M1nFAULT = 6;
  _M1DIR = 7;
  _M1PWM = 9;
  _M1CS = A0;

  _M2nSLEEP = 4;
  _M2nFAULT = 12;
  _M2DIR = 8;
  _M2PWM = 10;
  _M2CS = A1;
}

void MotorDriver::init()
{
  pinMode(_M1nSLEEP, OUTPUT);
  pinMode(_M2nSLEEP, OUTPUT);
  pinMode(_M1PWM, OUTPUT);
  pinMode(_M1nFAULT, INPUT_PULLUP);
  pinMode(_M1CS, INPUT);
  pinMode(_M1DIR, OUTPUT);
  pinMode(_M2DIR, OUTPUT);
  pinMode(_M2PWM, OUTPUT);
  pinMode(_M2nFAULT, INPUT_PULLUP);
  pinMode(_M2CS, INPUT);

  #if defined(REDBOARD_TURBO)
  pwm.setClockDivider(1, true);
  pwm.timer(0, 1, 4800, true);
  pwm.timer(1, 1, 4800, true);
  pwm.analogWrite(9, 0);
  pwm.analogWrite(10, 0);

  analogReadResolution(12);
  analogReadCorrection(0, 0x0800);
  #endif
}

void MotorDriver::setM1Speed(int speed)
{
  bool reverse = false;

  if(speed < 0)
  {
    speed = -speed;
    reverse = true;
  }
  if(speed > 1000)
  {
    speed = 1000;
  }
  #if defined(REDBOARD_TURBO)
  pwm.analogWrite(_M1PWM, speed);
  #endif

  if(reverse)
  {
    digitalWrite(_M1DIR, HIGH);
  }
  else
  {
    digitalWrite(_M1DIR, LOW);
  }
}

void MotorDriver::setM2Speed(int speed)
{
  bool reverse = false;

  if(speed < 0)
  {
    speed = -speed;
    reverse = true;
  }
  if(speed > 1000)
  {
    speed = 1000;
  }
  #if defined(REDBOARD_TURBO)
  pwm.analogWrite(_M2PWM, speed);
  #endif

  if(reverse)
  {
    digitalWrite(_M2DIR, HIGH);
  }
  else
  {
    digitalWrite(_M2DIR, LOW);
  }
}

void MotorDriver::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

// Return error status for motor 1
unsigned char MotorDriver::getM1Fault()
{
  return !digitalRead(_M1nFAULT);
}

// Return error status for motor 2
unsigned char MotorDriver::getM2Fault()
{
  return !digitalRead(_M2nFAULT);
}

// Enables the MOSFET driver for M1.
void MotorDriver::enableM1Driver()
{
  digitalWrite(_M1nSLEEP, HIGH);
}

// Enables the MOSFET driver for M2.
void MotorDriver::enableM2Driver()
{
  digitalWrite(_M2nSLEEP, HIGH);
}

// Enables the MOSFET drivers for both M1 and M2.
void MotorDriver::enableDrivers()
{
  enableM1Driver();
  enableM2Driver();
}

// Puts the MOSFET driver for M1 into sleep mode.
void MotorDriver::disableM1Driver()
{
  digitalWrite(_M1nSLEEP, LOW);
}

// Puts the MOSFET driver for M2 into sleep mode.
void MotorDriver::disableM2Driver()
{
  digitalWrite(_M2nSLEEP, LOW);
}

// Puts the MOSFET drivers for both M1 and M2 into sleep mode.
void MotorDriver::disableDrivers()
{
  disableM1Driver();
  disableM2Driver();
}

unsigned int MotorDriver::getM1CurrentReading()
{
  return analogRead(_M1CS);
}

unsigned int MotorDriver::getM2CurrentReading()
{
  return analogRead(_M2CS);
}

// Set voltage offset of M1 current reading at 0 speed.
void MotorDriver::calibrateM1CurrentOffset()
{
  setM1Speed(0);
  enableM1Driver();
  delay(1);
  MotorDriver::_offsetM1 = getM1CurrentReading();
}

// Set voltage offset of M2 current reading at 0 speed.
void MotorDriver::calibrateM2CurrentOffset()
{
  setM2Speed(0);
  enableM2Driver();
  delay(1);
  MotorDriver::_offsetM2 = getM2CurrentReading();
}

// Get voltage offset of M1 and M2 current readings.
void MotorDriver::calibrateCurrentOffsets()
{
  setSpeeds( 0, 0);
  enableDrivers();
  delay(1);
  MotorDriver::_offsetM1 = getM1CurrentReading();
  MotorDriver::_offsetM2 = getM2CurrentReading();
}

// Return M1 current value in milliamps using the gain value for the specific version.
unsigned int MotorDriver::getM1CurrentMilliamps()
{
  // 3.3v / 2048 ADC counts / gain mV per A
  // The 24v14, 18v18 and 24v18 results in 244 mA per count.
  // The 18v22 results in 488 mA per count.
  unsigned int mAPerCount = 3300000/4096/20;
  int reading = (getM1CurrentReading() - _offsetM1);
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

// Return M2 current value in milliamps using the gain value for the specific version.
unsigned int MotorDriver::getM2CurrentMilliamps()
{
  // 3.3v / 2048 ADC counts / gain mV per A
  // The 24v14, 18v18 and 24v18 results in 244 mA per count.
  // The 18v22 results in 488 mA per count.
  unsigned int mAPerCount = 3300000/4096/20;
  int reading = (getM2CurrentReading() - _offsetM2);
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

float MotorDriver::getFreq()
{
  return pwm.frequency(0);
}