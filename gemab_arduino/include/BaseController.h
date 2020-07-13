/*
  BaseController.h - Library for taking x and theta velocity and translating to dif drive.

  Released into the public domain.
*/
#ifndef BaseController_h
#define BaseController_h

#include "Arduino.h"
#include "PID.h"
#include "MotorDriver.h"
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

class BaseController
{
  public:
    BaseController();
    void init(ros::NodeHandle& nh, ADC& main_adc);
    void updateParameters(float width, float diameter, float max_v, int cpr, int rate, float reduction);
    void updatePID(float lKp, float lKi, float lKd, float rKp, float rKi, float rKd);
    void sendVelocity(float x, float theta);
    void updateRotation(float left, float right, long time);
    void getRotation(float* leftRad, float* rightRad);
    void updateOdom(nav_msgs::Odometry& odom);
    void updateTF(geometry_msgs::TransformStamped& odom_tf);
    void computeOutput(long dt);
    void updateOutput();
  private:
    float _width;
    float _diameterConst;
    float _max_v;
    int _CPR;
    float _CPM;
    int _rate;
    float _reduction;
    float _leftRad;
    float _rightRad;
    float _leftMeasRad;
    float _rightMeasRad;
    long _lastLeftPosition;
    long _lastRightPosition;
    float _xOutL;
    float _xOutR;
    float _lKp;
    float _lKi;
    float _lKd;
    float _rKp;
    float _rKi;
    float _rKd;
    uint32_t _period;
    float _odom_pose[3];
    float _odom_vel[3];

    PID* _leftPID;
    PID* _rightPID;
    MotorDriver* _motorDriver;
    ros::NodeHandle* _nh;
};

#endif
