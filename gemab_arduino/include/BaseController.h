/*
  BaseController.h - Library for taking x and theta velocity and translating to dif drive.

  Released into the public domain.
*/
#ifndef BaseController_h
#define BaseController_h

#include "Arduino.h"
#include <PID_v1.h>
#include "MotorDriver.h"
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

class BaseController
{
  public:
    BaseController();
    void init(ros::NodeHandle& nh);
    void updateParameters(float width, float diameter, float max_v, int cpr, int rate, float reduction);
    void updatePID(double lKp, double lKi, double lKd, double rKp, double rKi, double rKd);
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
    int _rate;
    float _reduction;
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
    float _odom_pose[3];
    float _odom_vel[3];
    float _last_lPerror;
    float _last_rPerror;
    float _lIerror;
    float _rIerror;

    PID* _leftPID;
    PID* _rightPID;
    MotorDriver* _motorDriver;
    ros::NodeHandle* _nh;
};

#endif
