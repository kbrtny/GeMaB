#pragma once

#include <Arduino.h>
#include <ADC.h>

#define MOTOR_MAX 2047


#define MOTOR_DEADBAND 5

class MotorDriver
{
  public:
    // CONSTRUCTORS
    MotorDriver();
    // PUBLIC METHODS
    void init(ADC& main_adc);
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    unsigned char getM1Fault(); // Get fault reading from M1.
    unsigned char getM2Fault(); // Get fault reading from M2.
    void enableM1Driver(); // Enables the MOSFET driver for M1.
    void enableM2Driver(); // Enables the MOSFET driver for M2.
    void enableDrivers(); // Enables the MOSFET drivers for both M1 and M2.
    void disableM1Driver(); // Puts the MOSFET driver for M1 into sleep mode.
    void disableM2Driver(); // Puts the MOSFET driver for M2 into sleep mode.
    void disableDrivers(); // Puts the MOSFET drivers for both M1 and M2 into sleep mode.
    unsigned int getM1CurrentReading();
    unsigned int getM2CurrentReading();
    void calibrateM1CurrentOffset();
    void calibrateM2CurrentOffset();
    void calibrateCurrentOffsets();
    unsigned int getM1CurrentMilliamps();
    unsigned int getM2CurrentMilliamps();

  protected:
    unsigned int _offsetM1;
    unsigned int _offsetM2;

  private:
    unsigned char _M1PWM;
    static const unsigned char _M1PWM_TIMER1_PIN = 9;
    unsigned char _M2PWM;
    static const unsigned char _M2PWM_TIMER1_PIN = 10;
    unsigned char _M1nSLEEP;
    unsigned char _M2nSLEEP;
    unsigned char _M1DIR;
    unsigned char _M2DIR;
    unsigned char _M1nFAULT;
    unsigned char _M2nFAULT;
    unsigned char _M1CS;
    unsigned char _M2CS;
    static boolean _flipM1;
    static boolean _flipM2;

    ADC* _adc;
    

};
