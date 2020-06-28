/* GeMaB base controller based on a Sparkfun RedBoard Turbo 
 * and a Pololu G2 motor driver shield
 * 
 * This contains a /cmd_vel subscriber and an /odom publisher
 * */



#define USE_REDBOARD_TURBO_USB
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>


#include <Encoder.h>
#include "BaseController.h"

#define ROBOT_WIDTH 370
#define ROBOT_WHEEL_DIAMETER 90
#define ROBOT_MAX_VELOCITY 100
#define ROBOT_CPR 28
#define CONTROLLER_UPDATE_RATE 200
#define USBCON

ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
BaseController bc;

char base_link[] = "/base_link";
char odom[] = "/odom";

MotorDriver md;
Encoder leftEnc(5,3);
Encoder rightEnc(11,13);

long leftPosition, rightPosition;
long current_time, elapsed_micros, last_called_time;

void cmd_velCb(const geometry_msgs::Twist& cmd_msg){
    float x = cmd_msg.linear.x;
    float theta = cmd_msg.angular.z;

    bc.sendVelocity(x, theta);
    bc.updateOutput();
}

ros::Subscriber<geometry_msgs::Twist> cvsub("cmd_vel", cmd_velCb );

double getdTheta(double dx, double dy)
{
  return tan(dx/dy);
}
void setup() {
  md.init();
  //md.calibrateCurrentOffsets();
  #if defined(REDBOARD_TURBO)
  analogReadCorrection(10, 2055);
  #endif

  bc.updateParameters(ROBOT_WIDTH, ROBOT_WHEEL_DIAMETER, ROBOT_MAX_VELOCITY, ROBOT_CPR, CONTROLLER_UPDATE_RATE);

  leftPosition = 0;
  rightPosition = 0;

  nh.initNode();
  nh.subscribe(cvsub);
  broadcaster.init(nh);
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  nh.spinOnce();
  last_called_time = micros();
}

void loop() {
  current_time = micros();
  elapsed_micros = current_time-last_called_time;
  if (elapsed_micros > (1000000/CONTROLLER_UPDATE_RATE - 1))
  {
    last_called_time = current_time;
    leftPosition = leftEnc.read();
    rightPosition = rightEnc.read();
    //SerialUSB.print(leftPosition);
    //SerialUSB.println(rightPosition);
    bc.updateRotation(leftPosition, rightPosition);
    //md.enableM2Driver();
    //md.setM2Speed(200);
    //SerialUSB.println(md.getFreq());
  }
  
  
  
  t.transform.translation.x = 1;
  t.transform.translation.y = 2;
  
  t.transform.rotation = tf::createQuaternionFromYaw(3);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  nh.spinOnce();

}
