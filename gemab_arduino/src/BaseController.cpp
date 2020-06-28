
#include "BaseController.h"


    
BaseController::BaseController()
{
  _leftRad = 0.0;
  _rightRad = 0.0;
  _lastLeftPosition = 0;
  _lastRightPosition = 0;
  _lKp = 0.0;
  _lKi = 0.0;
  _lKd = 0.0;
  _rKp = 0.0;
  _rKi = 0.0;
  _rKd = 0.0;
  _period = 48 - 1;

  _leftPID = new PID(&_leftMeasRad, &_leftRad, &_xOutL, this->_lKp, _lKi, _lKd, DIRECT);
  _rightPID = new PID(&_rightMeasRad, &_rightRad, &_xOutR, _rKp, _rKi, _rKd, DIRECT);
  _motorDriver = new MotorDriver();

}

void BaseController::init()
{
  _motorDriver->init();
  _motorDriver->calibrateCurrentOffsets();

  delay(10);
}

void BaseController::updateParameters(float width, float diameter, float max_v, int cpr, int rate)
{
  _width = width;
  _diameterConst = diameter * 3.14159;
  _CPR = cpr;
  _rate = rate;
  _max_v = max_v;
}

void BaseController::updatePID(double lKp, double lKi, double lKd, double rKp, double rKi, double rKd)
{
  _lKp = lKp;
  _lKi = lKi;
  _lKd = lKd;
  _rKp = rKp;
  _rKi = rKi;
  _rKd = rKd;
  _leftPID->SetTunings(_lKp, _lKi, _lKd);
  _rightPID->SetTunings(_rKp, _rKi, _rKd);
  
}

void BaseController::sendVelocity(float x, float theta)
{
  float diff = theta * _width / 2;
  //cheating a bit and using rotation/sec instead of rad/sec to save some math
  _leftRad = (x - diff)/_diameterConst;
  _rightRad = (x + diff)/_diameterConst;
}

void BaseController::updateRotation(float left, float right)
{
  long leftPosDelta = left - _lastLeftPosition;
  long rightPosDelta = right - _lastRightPosition;
  _lastLeftPosition = left;
  _lastRightPosition = right;
  _leftMeasRad = ((float)leftPosDelta/(float)_CPR)/(float)_rate;
  _rightMeasRad = ((float)rightPosDelta/(float)_CPR)/(float)_rate;
}

void BaseController::getRotation(float* leftRad, float* rightRad)
{
  *leftRad = _leftMeasRad;
  *rightRad = _rightMeasRad;
}

void BaseController::computeOutput()
{
  //_leftPID->Compute();
  //_rightPID->Compute();
}

void BaseController::setOutput()
{
  _motorDriver->setSpeeds(_xOutL, _xOutR);
}

void BaseController::updateOutput()
{
  if((_leftRad > 0.001)|(_leftRad < -0.001)){
    _motorDriver->enableM1Driver();
    if(_leftRad > 2.0)
    {
      _leftRad = 2.0;
    }
    else if(_leftRad < -2.0)
    {
      _leftRad = -2.0;
    }
    _motorDriver->setM1Speed(_leftRad*500);
    
  }else
  {
    _motorDriver->disableM1Driver();
    _motorDriver->setM1Speed(0);
  }
  if((_rightRad > 0.001)|(_rightRad < -0.001)){
    _motorDriver->enableM2Driver();
    if(_rightRad > 2.0)
    {
      _rightRad = 2.0;
    }
    else if(_rightRad < -2.0)
    {
      _rightRad = -2.0;
    }
    _motorDriver->setM2Speed(_rightRad*500);
    
  }else
  {
    _motorDriver->disableM2Driver();
    _motorDriver->setM2Speed(0);
  }
}
