#ifndef PID_h
#define PID_h

class PID
{
  public:
    PID(float* Input, float* Output, float* Setpoint);
    void Init(float outputLimit);
    void UpdateConstants(float P, float I, float D);
    void Compute();
  private:
    float _KP;
    float _KI;
    float _KD;
    float _last_Perror;
    float _Ierror;
    float _outputLimit;
    float *_Input;
    float *_Output;
    float *_Setpoint;


};
#endif