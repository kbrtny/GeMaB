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
#include <sensor_msgs/BatteryState.h>

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
#define BATT_VOLT_SCALE 3.3/4096*7.0455
#define BATTERY_VOLTAGE_PIN A2

#define LKP 2.0
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
sensor_msgs::BatteryState battery_state;

ros::Publisher odom_pub("odom", &odom);
ros::Publisher batt_pub("battery", &battery_state);

BaseController bc;
MotorDriver md;
Encoder leftEnc(5,3);
Encoder rightEnc(11,13);
ADC adc;

float pid_constants[6];
char odom_header_frame_id[] = "odom";
char odom_child_frame_id[] = "base_footprint";
char odom_tf_child_frame_id[] = "base_footprint";

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
  
  adc.adc0->setAveraging(4);
  adc.adc0->setResolution(12);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  bc.updatePID(LKP, LKI, LKD, RKP, RKI, RKD);
  bc.updateParameters(ROBOT_WIDTH, ROBOT_WHEEL_DIAMETER, ROBOT_MAX_VELOCITY, ROBOT_CPR, CONTROLLER_UPDATE_RATE, ROBOT_DRIVE_GEAR_REDUCTION);
  bc.init(nh, adc);

  leftPosition = 0;
  rightPosition = 0;

  nh.initNode();
  nh.subscribe(cvsub);
  nh.advertise(odom_pub);
  nh.advertise(batt_pub);
  tf_broadcaster.init(nh);
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id = odom_child_frame_id;
  odom_tf.header.frame_id = odom_header_frame_id;
  odom_tf.child_frame_id = odom_tf_child_frame_id;
  battery_state.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  battery_state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  
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
    
    

    odom_tf.header.stamp = stamp_now;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    
    battery_state.header.stamp = stamp_now;
    battery_state.voltage = adc.analogRead(BATTERY_VOLTAGE_PIN) * BATT_VOLT_SCALE;

    odom_pub.publish(&odom);
    tf_broadcaster.sendTransform(odom_tf);
    batt_pub.publish(&battery_state);

  }
  nh.spinOnce();

}
