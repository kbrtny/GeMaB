/*
  BaseController.h - Library for taking x and theta velocity and translating to dif drive.

  Released into the public domain.
*/
#ifndef BaseController_h
#define BaseController_h

#include "Arduino.h"
#include <PID_v1.h>
#include "MotorDriver.h"

class BaseController
{
  public:
    BaseController();
    void init();
    void updateParameters(float width, float diameter, float max_v, int cpr, int rate);
    void updatePID(double lKp, double lKi, double lKd, double rKp, double rKi, double rKd);
    void sendVelocity(float x, float theta);
    void updateRotation(float left, float right);
    void getRotation(float* leftRad, float* rightRad);
    void computeOutput();
    void setOutput();
    void updateOutput();
  private:
    float _width;
    float _diameterConst;
    float _max_v;
    int _CPR;
    int _rate;
    double _leftRad;
    double _rightRad;
    double _leftMeasRad;
    double _rightMeasRad;
    long _lastLeftPosition;
    long _lastRightPosition;
    double _xOutL;
    double _xOutR;
    double _lKp;
    double _lKi;
    double _lKd;
    double _rKp;
    double _rKi;
    double _rKd;
    uint32_t _period;

    PID* _leftPID;
    PID* _rightPID;
    MotorDriver* _motorDriver;
};

#endif
