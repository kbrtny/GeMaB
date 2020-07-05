
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
  _odom_pose[0] = 0.0;
  _odom_pose[1] = 0.0;
  _odom_pose[2] = 0.0;
  _odom_vel[0] = 0.0;
  _odom_vel[1] = 0.0;
  _odom_vel[2] = 0.0;
  //_last_RotTime = millis();

  _leftPID = new PID(&_leftMeasRad, &_xOutL, &_leftRad, _lKp, _lKi, _lKd, P_ON_E, DIRECT);
  _rightPID = new PID(&_rightMeasRad, &_xOutR, &_rightRad, _rKp, _rKi, _rKd, P_ON_E, DIRECT);
  _motorDriver = new MotorDriver();

}

void BaseController::init(ros::NodeHandle& nh)
{
  _nh = &nh;
  _motorDriver->init();
  _motorDriver->calibrateCurrentOffsets();
  _leftPID->SetOutputLimits(-MOTOR_MAX, MOTOR_MAX);
  _rightPID->SetOutputLimits(-MOTOR_MAX, MOTOR_MAX);
  //_leftPID->SetSampleTime(20);
  //_rightPID->SetSampleTime(20);
  _leftPID->SetMode(AUTOMATIC);
  _rightPID->SetMode(AUTOMATIC);
  delay(10);
}

void BaseController::updateParameters(float width, float diameter, float max_v, int cpr, int rate, float reduction)
{
  _width = width;
  _diameterConst = diameter * 3.14159 / reduction;
  _CPR = cpr;
  _rate = rate;
  _max_v = max_v;
  _reduction = reduction;
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
  _last_lPerror = 0;
  _last_rPerror = 0;
  _lIerror = 0;
  _rIerror = 0;
  
}

void BaseController::sendVelocity(float x, float theta)
{
  char log_msg[50];
  float diff = theta * _width / 2;
  //cheating a bit and using rotation/sec instead of rad/sec to save some math
  _leftRad = (x - diff) / _diameterConst;
  _rightRad = (x + diff) / _diameterConst;
  sprintf(log_msg, "Des Rad: %f , %f", _leftRad, _rightRad);
  _nh->loginfo(log_msg);
}

void BaseController::updateRotation(float left, float right, long diff_time)
{
  char log_msg[50];
  long leftPosDelta = left - _lastLeftPosition;
  long rightPosDelta = right - _lastRightPosition;
  float deltaTimeMinute = (float)diff_time / 60000.0;
  float x, y, v, w, delta_s, delta_theta;
  _lastLeftPosition = left;
  _lastRightPosition = right;
  _leftMeasRad = ((float)leftPosDelta/(float)_CPR)/deltaTimeMinute;
  _rightMeasRad = ((float)rightPosDelta/(float)_CPR)/deltaTimeMinute;
  sprintf(log_msg, "Meas Rad: %f , %f", _leftMeasRad, _rightMeasRad);
  //_nh->loginfo(log_msg);
  delta_s = _diameterConst * (float)_CPR * ((float)leftPosDelta + (float)rightPosDelta) / 2.0;
  delta_theta = _diameterConst * (float)_CPR * ((float)rightPosDelta - (float)leftPosDelta) / _width;

  x = cos(delta_theta) * delta_s;
  y = -sin(delta_theta) * delta_s;
  _odom_pose[0] += cos(_odom_pose[2]) * x - sin(_odom_pose[2]) * y;
  _odom_pose[1] += sin(_odom_pose[2]) * x + cos(_odom_pose[2]) * y;
  _odom_pose[2] += delta_theta;

  v = delta_s / diff_time;
  w = delta_theta / diff_time;

  _odom_vel[0] = v;
  _odom_vel[1] = 0.0;
  _odom_vel[2] = w;
  //test
}

void BaseController::updateOdom(nav_msgs::Odometry& odom)
{
  odom.pose.pose.position.z = 0;
  odom.pose.pose.position.x = _odom_pose[0];
  odom.pose.pose.position.y = _odom_pose[1];
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(_odom_pose[2]);

  odom.twist.twist.linear.x = _odom_vel[0];
  odom.twist.twist.angular.z = _odom_vel[2];
}

void BaseController::computeOutput(long dt)
{
  char log_msg[50];
  float lPerror = _leftRad - _leftMeasRad;
  float rPerror = _rightRad - _rightMeasRad;
  float lDerror = (_leftRad - _last_lPerror);
  float rDerror = (_rightRad - _last_rPerror);
  _last_lPerror = lPerror;
  _last_rPerror = rPerror;
  _lIerror += lPerror;
  _rIerror += rPerror;

  _xOutL += _lKp * lPerror + _lKi * _lIerror + _lKd * lDerror;
  _xOutR += _rKp * rPerror + _rKi * _rIerror + _rKd * rDerror;
  if(_xOutL > MOTOR_MAX)
  {
    _xOutL = MOTOR_MAX;
    _lIerror = 0;
  }else if(_xOutL < -MOTOR_MAX)
  {
    _xOutL = -MOTOR_MAX;
    _lIerror = 0;
  }
  if(_xOutR > MOTOR_MAX)
  {
    _xOutR = MOTOR_MAX;
    _rIerror = 0;
  }else if(_xOutR < -MOTOR_MAX)
  {
    _xOutR = -MOTOR_MAX;
    _rIerror = 0;
  }
  //int l, r;
  //l = _leftPID->Compute();
  //r = _rightPID->Compute();
  sprintf(log_msg, "IO: %f , %f, %f", _leftMeasRad, _leftRad, _xOutL);
  _nh->loginfo(log_msg);
}


void BaseController::updateOutput()
{
  if((_xOutL > MOTOR_DEADBAND)|(_xOutL < -MOTOR_DEADBAND)){
    _motorDriver->enableM1Driver();
    _motorDriver->setM1Speed(_xOutL);
    
  }else
  {
    _motorDriver->disableM1Driver();
    _motorDriver->setM1Speed(0);
  }
  if((_xOutR > MOTOR_DEADBAND)|(_xOutR < -MOTOR_DEADBAND)){
    _motorDriver->enableM2Driver();
    _motorDriver->setM2Speed(_xOutR);
    
  }else
  {
    _motorDriver->disableM2Driver();
    _motorDriver->setM2Speed(0);
  }
}
