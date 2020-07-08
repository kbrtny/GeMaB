#include "PID.h"

PID::PID(float* Input, float* Output, float* Setpoint)
{
  _Input = Input;
  _Output = Output;
  _Setpoint = Setpoint;
  _last_Perror = 0;
  _Ierror = 0;
}

void PID::Init(float outputLimit)
{
  _outputLimit = outputLimit;
  if (*_Output > _outputLimit)
  {
    *_Output = _outputLimit;
  }else if (*_Output < -_outputLimit)
  {
    *_Output = -_outputLimit;
  }
  _last_Perror = 0;
  _Ierror = 0;

}

void PID::UpdateConstants(float P, float I, float D)
{
  _KP = P;
  _KI = I;
  _KD = D;
}

void PID::Compute()
{
  float Perror = *_Setpoint - *_Input;
  float Derror = *_Setpoint - _last_Perror;
  _last_Perror = Perror;
  _Ierror += Perror;

  *_Output += _KP * Perror + _KI * _Ierror + _KD * Derror;
  if(*_Output > _outputLimit)
  {
    *_Output = _outputLimit;
    _Ierror = 0;
  }else if(*_Output < -_outputLimit)
  {
    *_Output = -_outputLimit;
    _Ierror = 0;
  }
}