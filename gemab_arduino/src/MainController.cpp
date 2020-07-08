/* GeMaB base controller based on a Sparkfun RedBoard Turbo 
 * and a Pololu G2 motor driver shield
 * 
 * This contains a /cmd_vel subscriber and an /odom publisher
 * */



//#define USE_REDBOARD_TURBO_USB
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#if defined (TEENSY)
  #define ENCODER_OPTIMIZE_INTERRUPTS
#endif
#include <Encoder.h>
#include "BaseController.h"

#define ROBOT_WIDTH 0.37
#define ROBOT_WHEEL_DIAMETER 0.09
#define ROBOT_MAX_VELOCITY 100
#define ROBOT_CPR 28
#define CONTROLLER_UPDATE_RATE 10
#define ROBOT_DRIVE_GEAR_REDUCTION 40

#define LKP 100.0
#define LKI 0.0
#define LKD 0.0
#define RKP LKP
#define RKI LKI
#define RKD LKD

// ROS objects
ros::NodeHandle nh;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;
nav_msgs::Odometry odom;

ros::Publisher odom_pub("odom", &odom);

BaseController bc;
MotorDriver md;
Encoder leftEnc(5,3);
Encoder rightEnc(11,13);

float pid_constants[6];
char odom_header_frame_id[] = "odom";
char odom_child_frame_id[] = "base_footprint";
char odom_tf_child_frame_id[] = "base_link";

long leftPosition, rightPosition;
long current_time, elapsed_micros, last_called_time;

void cmd_velCb(const geometry_msgs::Twist& cmd_msg){
    float x = cmd_msg.linear.x;
    float theta = cmd_msg.angular.z;
    bc.sendVelocity(x, theta);
}

ros::Subscriber<geometry_msgs::Twist> cvsub("cmd_vel", cmd_velCb );

void setup() {
  nh.initNode();

  bc.updatePID(LKP, LKI, LKD, RKP, RKI, RKD);
  bc.updateParameters(ROBOT_WIDTH, ROBOT_WHEEL_DIAMETER, ROBOT_MAX_VELOCITY, ROBOT_CPR, CONTROLLER_UPDATE_RATE, ROBOT_DRIVE_GEAR_REDUCTION);
  bc.init(nh);

  leftPosition = 0;
  rightPosition = 0;

  nh.initNode();
  nh.subscribe(cvsub);
  nh.advertise(odom_pub);
  tf_broadcaster.init(nh);
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id = odom_child_frame_id;
  odom_tf.header.frame_id = odom_header_frame_id;
  odom_tf.child_frame_id = odom_tf_child_frame_id;
  
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  last_called_time = micros();
}

void loop() {
  current_time = micros();
  elapsed_micros = current_time-last_called_time;
  if (elapsed_micros > (1000000/CONTROLLER_UPDATE_RATE - 1))
  {
    last_called_time = current_time;
    
    ros::Time stamp_now = nh.now();
    odom.header.stamp = stamp_now;
    
    leftPosition = leftEnc.read();
    rightPosition = rightEnc.read();

    bc.updateRotation(leftPosition, rightPosition, elapsed_micros);
    bc.computeOutput(elapsed_micros);
    bc.updateOutput();
    bc.updateOdom(odom);
    
    odom_pub.publish(&odom);

    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    odom_tf.header.stamp = nh.now();  
    tf_broadcaster.sendTransform(odom_tf);
    

  }
  nh.spinOnce();

}
