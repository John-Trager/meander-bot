/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <Encoder.h>
#include <string.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
Encoder leftEncoder(2, 4);
Encoder rightEncoder(3, 5);

// TODO: change to use a different message type...
// probably something like: double [angular momentum, angular position]
std_msgs::Int32 encoder_ticks_right;
std_msgs::Int32 encoder_ticks_left;


ros::Publisher pub_encoder_right("encoder_ticks_right", &encoder_ticks_right);
ros::Publisher pub_encoder_left("encoder_ticks_left", &encoder_ticks_left);


ros::NodeHandle  nh;

// ROS Subscriber setup to reset both encoders to zero
void resetCallback( const std_msgs::Empty& reset)
{
  // reset both back to zero.
  leftEncoder.write(0);
  rightEncoder.write(0);
  nh.loginfo("Reset both wheel encoders to zero");
}

ros::Subscriber<std_msgs::Empty> sub_reset("reset", resetCallback);

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  Serial.println("Starting Encoder Service");

  nh.initNode();
  nh.advertise(pub_encoder_right);
  nh.advertise(pub_encoder_left);

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

long leftPos  = -999;
long rightPos  = -999;

void loop() {
  long newLeftPos = leftEncoder.read();
  long newRightPos = rightEncoder.read();

  encoder_ticks_right.data = newRightPos;
  encoder_ticks_left.data = newLeftPos;

  pub_encoder_right.publish(&encoder_ticks_right);
  pub_encoder_left.publish(&encoder_ticks_left);
  nh.spinOnce();


  if (newLeftPos != leftPos || newRightPos != rightPos) {
    leftPos = newLeftPos;
    rightPos = newRightPos;
  }
  
  // need delay, otherwise could cause issues with rosserial
  delay(10);
}
