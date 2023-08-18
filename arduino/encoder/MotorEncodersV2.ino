#include "encodermeanderbot.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

const int encoder_resolution = 540;
ros::NodeHandle nh;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
meanderbot::Encoder leftEncoder(nh, 2, 4, encoder_resolution);
meanderbot::Encoder rightEncoder(nh, 3, 5, encoder_resolution);

std_msgs::Float64 encoder_left_ang_vel;
std_msgs::Float64 encoder_left_ang_pos;

std_msgs::Float64 encoder_right_ang_vel;
std_msgs::Float64 encoder_right_ang_pos;

ros::Publisher pub_encoder_left_vel("encoder_left_ang_vel", &encoder_left_ang_vel);
ros::Publisher pub_encoder_left_pos("encoder_left_ang_pos", &encoder_left_ang_pos);

ros::Publisher pub_encoder_right_vel("encoder_right_ang_vel", &encoder_right_ang_vel);
ros::Publisher pub_encoder_right_pos("encoder_right_ang_pos", &encoder_right_ang_pos);


// ROS Subscriber setup to reset both encoders to zero
void resetCallback( const std_msgs::Empty& reset);

ros::Subscriber<std_msgs::Empty> sub_reset("reset", resetCallback);

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  Serial.println("Starting Encoder Service");

  nh.initNode();
  nh.advertise(pub_encoder_left_vel);
  nh.advertise(pub_encoder_left_pos);
  nh.advertise(pub_encoder_right_vel);
  nh.advertise(pub_encoder_right_pos);

  nh.subscribe(sub_reset);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  Serial.println("Initialize Meamderbot Wheel Encoders");
  nh.loginfo("Initialize Meamderbot Wheel Encoders");
  std_msgs::Empty reset;
  resetCallback(reset);
  delay(1);
}

meanderbot::JointState leftState{0,0};
meanderbot::JointState rightState{0,0};

void loop() {
  meanderbot::JointState newLeftState = leftEncoder.jointState();
  meanderbot::JointState newRightState = rightEncoder.jointState();

  encoder_left_ang_pos.data = newLeftState.angular_position_;
  encoder_left_ang_vel.data = newLeftState.angular_velocity_;

  encoder_right_ang_pos.data = newRightState.angular_position_;
  encoder_right_ang_vel.data = newRightState.angular_velocity_;

  pub_encoder_left_pos.publish(&encoder_left_ang_pos);
  pub_encoder_left_vel.publish(&encoder_left_ang_vel);

  pub_encoder_right_pos.publish(&encoder_right_ang_pos);
  pub_encoder_right_vel.publish(&encoder_right_ang_vel);
  nh.spinOnce();

  if (newLeftState != leftState || newRightState != rightState) {
    leftState = newLeftState;
    rightState = newRightState;
    // TODO: maybe later update and publish the values in here so we don't send duplicate values??
  }
  
  // need delay, otherwise could cause issues with rosserial
  delay(10);
}

// ROS Subscriber setup to reset both encoders to zero
void resetCallback( const std_msgs::Empty& reset)
{
  // reset both back to zero.
  leftEncoder.write(0);
  rightEncoder.write(0);
  nh.loginfo("Reset both wheel encoders to zero");
}
